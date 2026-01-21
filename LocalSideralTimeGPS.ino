#include <WiFi.h>
#include <math.h>

// These are here to be manageable in PC simulation mode
#include "driver/uart.h"
int const gpsPin= 4;
int gpsGetData(char *b, int size) { 
    int r= uart_read_bytes(UART_NUM_1, b, size, 10);
    b[r]= 0;
    printf("%s", b);
    return r;
}
bool gpsBegin()
{
    const uart_config_t uart_config = { .baud_rate= 115200, .data_bits= UART_DATA_8_BITS, .parity= UART_PARITY_DISABLE, .stop_bits= UART_STOP_BITS_1, .flow_ctrl= UART_HW_FLOWCTRL_DISABLE };
    uart_driver_install(UART_NUM_1, 1024*2, 1024*2, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, -1, gpsPin, -1, -1);
    return true;
}
namespace CGPS {
    // Typical data comming form the gps at 9600 baud:
    // Info on visible satelites...
    //$BDGSV,1,1,03,14,38,234,28,24,65,289,28,41,,,21,0*7C
    // position: time 16:23:04 UTC, 45°44.97320N 004°50.24199,E 152.8 alt
    //$GNGGA,162304.000,4544.97320,N,00450.24199,E,1,07,1.1,152.8,M,50.2,M,,*46
    // lat, long time. As above
    //$GNGLL,4544.97320,N,00450.24199,E,162304.000,A,A*4D
    // more satelite infos
    //$GNGSA,A,3,12,15,19,25,32,,,,,,,,2.4,1.1,2.1,1*3D
    // more satelite infos
    //$GNGSA,A,3,14,24,,,,,,,,,,,2.4,1.1,2.1,4*32
    // speed/direction information
    //$GNRMC,162304.000,A,4544.97320,N,00450.24199,E,0.00,0.00,141025,,,A,V*03
    // Track & Speed over ground
    //$GNVTG,0.00,T,,M,0.00,N,0.00,K,A*23
    // time and date... time date as 14th oct 2025 with local time offset (00)
    //$GNZDA,162304.000,14,10,2025,00,00*4B
    // 6 satelite info from gps. as in $BDGSV, the 2,1 and 2,2 means 2 lines, line 1 and line 2....
    //$GPGSV,2,1,06,10,,,26,12,69,244,33,15,23,177,27,19,34,059,27,0*5C
    //$GPGSV,2,2,06,25,35,246,32,32,23,316,28,0*6D
    // random text...
    //$GPTXT,01,01,01,ANTENNA OK*35

    // We will extract info from
    //$GNGGA,162304.000,4544.97320,N,00450.24199,E,1,07,1.1,152.8,M,50.2,M,,*46
    //$GNGLL,4544.97320,N,00450.24199,E,162304.000,A,A*4D
    //$GNZDA,162304.000,14,10,2025,00,00*4B

    float latitude=NAN, longitude= NAN, altitude=NAN; // Will be valid once hasPosInfo is true. angles in radians, altitude in meters...
    char latitudetxt[30], longitudetxt[30];
    int h, m, s, D, M, Y; // will be valid once hasTimeInfo is true. Y is 2025
    bool hasPosInfo= false, hasTimeInfo= false, talking= false;
	bool waitGPS= false;                    // true if we are waiting on GPS init
    static int readint(char *&s, int nbchr) // read n chr in an int. return -1 if they was not n numerical chrs.... s points at end of number at the end
    { int res= 0; while (--nbchr>=0) { if (*s<'0' || *s>'9') return -1; res= res*10+*s++-'0'; } return res; }
    static float readAngle(char *&s, int nb, char *cpy) // read an angle in ddmm.frac_part form where frac_part has an undefined length. result in rad. returns 1000 on error. s points at end of number at the end
    {
        char *start= s;
        int d= readint(s,nb); if (d==-1) return NAN;
        int m= readint(s,2); if (m==-1) return NAN;
        float dec= 0.0f;
        if (*s=='.')
        {   s++; float f= 0.1f;
            while (true) { int v= readint(s,1); if (v<0) break; dec+= v*f; f/=10.0f; }
        }
        float r= d+(m+dec)/60.0f;
        sprintf(cpy, "%d:%d\'%.1f", d, m, dec*60.0f);
        return r;
    }
    static bool skipComa(char *&s, char *end) // find a coma and skip it. return true if they was a coma before end...
    {
        while (*s!=',' && s<end) s++; if (*s!=',') return false; s++; return true;
    }
    struct { char l[100]; } ls[10];
    // Task that reads incoming data from GPS and populate the data
    static void getInfo(void *) 
    { 
        for (int i=0; i<10; i++) ls[i].l[0]= 0;
        strcpy(latitudetxt, "unknown");strcpy(longitudetxt, "unknown");
        char b[512]; int sze= 0;
        while (true)
        {
            // if ((hasPosInfo && hasTimeInfo) || !waitGPS) { gpsDone(); return; }// end of task. frees all that is needed...
            int r= gpsGetData(b+sze, sizeof(b)-sze-1);
            if (r<0) continue;
            // b[sze+r]= 0; printf("%s", b+sze); // temporary...
            sze+= r; b[sze]=0;
            while (true)
            {
                char *l= strchr(b, '$'); if (l==nullptr) break; talking= true;
                char *le= strchr(l, '\n'); if (le==nullptr) break;
                for (int i=0; i<9; i++) strcpy(ls[i].l, ls[i+1].l); memset(ls[9].l, 0, 100); memcpy(ls[9].l, l, le-l-1);
                // here, we do have a full line! garantied!
                if (memcmp(l, "$GNGGA", 6)==0) { skipComa(l,le); goto pos; } // skip 2 ',' and then read as in $GNGLL
                if (memcmp(l, "$GNGLL", 6)==0) 
                { pos: 
                    if (!skipComa(l,le)) goto next; // skip 1 , and then read 4544.97320,N,00450.24199,E,1,07,1.1,152.8,M,50.2,M,,*46 where 152 is the altitude...
                    latitude= readAngle(l,2, latitudetxt); if (!skipComa(l,le)) goto next; if (*l!='N') latitude= -latitude;
                    if (!skipComa(l,le)) goto next;
                    longitude= readAngle(l,3, longitudetxt); if (!skipComa(l,le)) goto next; if (*l=='E') longitude= -longitude;
                    for (int i=0; i<4; i++) if (!skipComa(l,le)) goto next; // skip 4 commas...
                    if (*l<'0' || *l>'9') goto next;
                    altitude= 0; while (*l>='0' && *l<='9') altitude= altitude*10+*l++-'0';
                    printf("pos %f %f %f\n", latitude, longitude, altitude);
                    hasPosInfo= true; goto next;
                }
                if (memcmp(l, "$GNZDA", 6)==0) 
                { 
                    if (!skipComa(l,le)) goto next; // skip 1 , and then read 162304.000,14,10,2025,00,00*4B (hms, d, m y)
                    h= readint(l,2); if (h==-1) goto next; m= readint(l,2); if (m==-1) goto next; s= readint(l,2); if (s==-1) goto next;
                    if (!skipComa(l,le)) goto next;
                    D= readint(l,2); if (D==-1) goto next;
                    if (!skipComa(l,le)) goto next;
                    M= readint(l,2); if (M==-1) goto next;
                    if (!skipComa(l,le)) goto next;
                    Y= readint(l,4); if (Y==-1) goto next;
                    printf("time %d-%d-%d %d:%d:%d\n", Y, M, D, h, m, s);
                    hasTimeInfo= true; goto next;
                }
                if (memcmp(l, "$GNRMC", 6)==0) // $GNRMC,145250.200,A,4544.97860,N,00450.23108,E,0.00,0.00,101225,,,A*78
                { 
                    if (!skipComa(l,le)) goto next; // skip 1 , and then read 162304.000,14,10,2025,00,00*4B (hms, d, m y)
                    h= readint(l,2); if (h==-1) goto next; m= readint(l,2); if (m==-1) goto next; s= readint(l,2); if (s==-1) goto next;
                    for (int i=0; i<8; i++) if (!skipComa(l,le)) goto next;
                    D= readint(l,2); if (D==-1) goto next; printf("D:%d ", D);
                    M= readint(l,2); if (M==-1) goto next; printf("M:%d ", M);
                    Y= readint(l,2); if (Y==-1) goto next; Y+= 2000;  printf("Y:%d\n", Y);
                    printf("time %d-%d-%d %d:%d:%d\n", Y, M, D, h, m, s);
                    hasTimeInfo= true; goto next;
                }
                next: memcpy(b, le+1, b+sizeof(b)-le-1); sze-= int(le+1-b);
            }
        }
    }

    // return true if gps system was started...
    bool begin() { if (!gpsBegin()) return false; xTaskCreate(getInfo, "GPS", 4096, NULL, 2, NULL); return true; }

    // Compute Local Sidereal Time (in hours)
        static double normalize24(double hours) { hours = fmod(hours, 24.0); if (hours < 0) hours += 24.0; return hours; }
    double localSiderealTime()
    {
        // Compute Julian Date from UTC date and time
        double utHours= h+m/60.0+s/3600.0;

        int month= M, year= Y;
        if (month<=2) { year -= 1; month += 12; }
        int A = year / 100;
        int B = 2 - A + A / 4;
        double JD = floor(365.25 * (year + 4716)) + floor(30.6001 * (month + 1)) + D + B - 1524.5;
        JD -= 2451545;
        double GMST_0 = 18.697374558 + 24.06570982441908*JD; // GMST at 0h UT (in hours)
        double GMST = GMST_0 + 1.00273790935*utHours; // GMST at given UT
        GMST = normalize24(GMST);
        double LST = GMST + longitude*(12.0/180.0); // Convert to local sidereal time
        LST = normalize24(LST);
        return LST;
    }

};

#include <TM1637Display.h>
#define TMC1DIO 6
#define TMC1CLK 5
#define TMC2DIO 8
#define TMC2CLK 7
// Let us note that the "data" to position is sem inverted. if you send in data 012345 the display is 210543, so inverted by packs of 3.
TM1637Display TMC1(TMC1CLK, TMC1DIO);
TM1637Display TMC2(TMC2CLK, TMC2DIO);
uint8_t *descramble(uint8_t *d, uint8_t *s) // undo the 
 {
    d[0]= s[2]; d[1]= s[1]; d[2]= s[0];
    d[3]= s[5]; d[4]= s[4]; d[5]= s[3];
    return d;
 }
 
uint8_t noGPS[] = { 0, SEG_D, SEG_D, SEG_D, SEG_D, SEG_D };

int i=0;
void mainTask(void*) 
{
    while (true)
    {
        uint8_t t[6]; 
        TMC1.setBrightness(0x04);
        TMC2.setBrightness(0x04);
        if (CGPS::hasTimeInfo)
        {
            uint8_t data[6];
            data[2]= TMC1.encodeDigit(CGPS::h/10);
            data[1]= TMC1.encodeDigit(CGPS::h%10) | 0x80;
            data[0]= TMC1.encodeDigit(CGPS::m/10);
            data[5]= TMC1.encodeDigit(CGPS::m%10) | 0x80;
            data[4]= TMC1.encodeDigit(CGPS::s/10);
            data[3]= TMC1.encodeDigit(CGPS::s%10);
            TMC2.setSegments(data, 6);
            if (CGPS::hasPosInfo)
            {
                double h= CGPS::localSiderealTime();
                data[2]= TMC1.encodeDigit(int(h/10));
                data[1]= TMC1.encodeDigit(int(fmod(h, 10))) | 0x80;
                h= (h-int(h))*60;
                data[0]= TMC1.encodeDigit(int(h/10));
                data[5]= TMC1.encodeDigit(int(fmod(h, 10))) | 0x80;
                h= (h-int(h))*60;
                data[4]= TMC1.encodeDigit(int(h/10));
                data[3]= TMC1.encodeDigit(int(fmod(h, 10)));
                TMC1.setSegments(data, 6);
            } else {
                descramble(t, noGPS);
                TMC1.setSegments(t, 6);
            }
        } else {
            descramble(t, noGPS);
            TMC1.setSegments(t, 6);
            TMC2.setSegments(t, 6);
        }
        { uint8_t t= noGPS[0]; memcpy(noGPS, noGPS+1, 5); noGPS[5]= t; } // loop!

        delay(200);
    }
}

NetworkServer server(80);

void setup() 
{
  CGPS::begin();
  xTaskCreate(mainTask, "main", 4096, NULL, 2, NULL);
  WiFi.softAP("GpsSideral"); // , "abcdefg");
  server.begin();
}


static char const page[]=
"HTTP/1.1 200 OK\n"\
"Content-type:text/html\n\n"\
"<!DOCTYPE html>"\
"<html lang=\"en\">"\
"<head>"\
"    <meta charset=\"UTF-8\">"\
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"\
"    <title>Local Sideral Clock</title>"\
"    <style>"\
"        body {"\
"            background-color: black;"\
"            color: white;"\
"            font-family: Arial, sans-serif;"\
"            text-align: center;"\
"            margin: 0;"\
"            padding: 20px;"\
"        }"\
"        header {"\
"            border-bottom: 1px solid #333;"\
"            padding-bottom: 10px;"\
"        }"\
"    </style>"\
"</head>"\
"<body>"\
"    <header>"\
"        <h1>The Local Sideral Time is<br>%d:%d:%d</h1>"\
"        <h1>UTC time is<br>%d:%d:%d</h1>"\
"        <h1>Latitude<br>%s = %f</h1>"\
"        <h1>Longitude<br>%s = %f</h1>"\
"        <h1>Altitude<br>%dm</h1>"\
"    </header>"\
"    <main>"\
"      <button onclick=\"location.reload()\">Refresh Page</button>"\
"    </main>"\
"</body>"\
"</html>\n\n";
static char const page2[]=
"HTTP/1.1 200 OK\n"\
"Content-type:text/html\n\n"\
"<!DOCTYPE html>"\
"<html lang=\"en\">"\
"<head>"\
"    <meta charset=\"UTF-8\">"\
"    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"\
"    <title>Local Sideral Clock</title>"\
"    <style>"\
"        body {"\
"            background-color: black;"\
"            color: white;"\
"            font-family: Arial, sans-serif;"\
"            text-align: center;"\
"            margin: 0;"\
"            padding: 20px;"\
"        }"\
"        header {"\
"            border-bottom: 1px solid #333;"\
"            padding-bottom: 10px;"\
"        }"\
"    </style>"\
"</head>"\
"<body>"\
"    <header>"\
"        <h1>GPS pas encore actif, reessayez dans quelques secondes</h1>"\
"    </header>"\
"    <main>"\
"      <button onclick=\"location.reload()\">Refresh Page</button>"\
"    </main>"\
"    <header>"\
"        <h1>Last GPS data</h1>"\
"    </header>"\
"    <main>"\
"      <p>%s<br>%s<br>%s<br>%s<br>%s<br>%s<br>%s<br>%s<br>%s<br>%s<br></p>"\
"    </main>"\
"</body>"\
"</html>\n\n";
static char pagebuf[2048]; // global alloc char buffer for  writing page and sending it..

void loop() 
{
  NetworkClient client = server.accept();  // listen for incoming clients

  if (client) {                     // if you get a client,
    printf("New Client.\n");  // print a message out the serial port
    String currentLine = "";        // make a String to hold incoming data from the client
    while (client.connected()) {    // loop while the client's connected
      if (client.available()) {     // if there's 123bytes to read from the client,
        char c = client.read();     // read a byte, then
        Serial.write(c);            // print it out the serial monitor
        if (c == '\n') {            // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            if (CGPS::hasPosInfo)
            {
                double lst= CGPS::localSiderealTime();
                int lsth= int(lst);
                lst= (lst-int(lst))*60;
                int lstm= int(lst);
                lst= (lst-int(lst))*60;
                int lsts= int(lst);
                sprintf(pagebuf, page, lsth, lstm, lsts, CGPS::h, CGPS::m, CGPS::s, CGPS::latitudetxt, CGPS::latitude, CGPS::longitudetxt, CGPS::longitude, int(CGPS::altitude));
                client.print(pagebuf);
                // break out of the while loop:
                break;
            } else {
                sprintf(pagebuf, page2, CGPS::ls[0].l, CGPS::ls[1].l, CGPS::ls[2].l, CGPS::ls[3].l, CGPS::ls[4].l, CGPS::ls[5].l, CGPS::ls[6].l, CGPS::ls[7].l, CGPS::ls[8].l, CGPS::ls[9].l);
                client.print(pagebuf);
                break;
            }
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

      }
    }
    // close the connection:
    client.stop();
    printf("Client Disconnected.\n");
  }
}
