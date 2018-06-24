/*************************************************************************************/
/*  MOBISENS 0.7.6

   mobile sensor apparatus    for particle matter, NO2 CO Gas,
   Temperature, AirPressure, Humidity, GPS Position on Display
   and in future
   transmit by WLAN or Lorawan or store on SD-Card if no Connection Available

   Hardware:
   NodeMCU, SDS011, BME280, CJMCU-4541(MiCS-4514),GPS,
   [CD-Card-Reader, Lorawan-transmitter]


   Done:
   OLED: simple Text Interface
   GPS: LAT, LON, Altitude, UTC Date and Time, speed, direction,
   BME280: Temperature, AirPressure, Humidity, DewPoint, SeaLevelPressure, AltitudebyAirPressure
   SDS011: PM2.5, PM10
   CJMCU-4514: no2 co calibration by calculating R0 R0 based on Rs and official values (evaluation tbd)


   open:
   OLED: improve numbers and figures on several screens
   SDS011: debug occurrences of no-sleep
   CJMCU: verify calculation to real data
   send data to luftdaten.info, hackair, Airqn, opensense.net, own server, ...



   all based on code, examples and licenses of arduino/github libraries from

   ssd1306 SimpleDemo v Eichhorn,
   BME280.ino,
   Tinygps
   SDS011  v Rajko Zschiegner, aus Beta und library
   humidity compensation for SDS011:
     https://github.com/piotrkpaul/esp8266-sds011/blob/masteur/sds011_nodemcu/sds011_nodemcu.ino
     Opengeiger.de       Kompensation _c Berechnung
   ADS1115 Adafruit examples
   CJMCU-4541(MiCS-4514)shawn hymel, Roland Ortner myscope.net, Marcel belledin (oklabkoELN)

   This Program is under GPL3 License.  no warranty. use it on your own risk.

  /************************************************************************************************************/



//---ESP8266 WIFI ----------------------------------------------------------------------------

#include <ESP8266WiFi.h>
/**/

//---Portexpander----------------------------------------------------------------------------

#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//-GPS--------------------------------------------------------------------------------------

#include <SoftwareSerial.h>
#include <TinyGPS.h>

SoftwareSerial ss(D5, D6);  //<----------------------------RX D5 <-(TXGPS), TX D6 <-(RX GPS)
TinyGPS gps;

//-I2C ----------------------------------------------------------------------------------
// For a connection via I2C using Wire include
#include <Wire.h>    // Only needed for Arduino 1.6.5 and earlier    i2c 


//-SSD1306 OLED -------------------------------------------------------------------------

#include "SSD1306.h"             // alias for `#include "SSD1306Wire.h"`

// Initialize the OLED display using Wire library
SSD1306  display(0x3c, D4, D3);  // 0x3c,D4,D3   <--------------------I2c-Address, SDA, SCL



//-Texte --------------------------------------------------------------------------------

String Textzeilehm;     //Stunde:Minute
String TextzeileT;      //Temperatur
String TextzeileH;      //Diff Höhe GPS und Barometer
String TextzeileG;      //GPS
String TextzeileGd;     //GPSDatumZeit
String TextzeileSDS;    //SDS011
String TextzeileSDSc;   //SDS011compensiert
String TextzeileMiCS;   //MiCswerte aus dem ADS   NO2
String TextzeileMiCSC;  //MiCswerte aus dem ADS  CO
String TextzeileNOx;    // echte ?? werte
String TextzeileCO;     // echte ?? werte
String Textzeileppm;
String sek;             // Errorcode lesen und sek
String s_err;
//------GPS-------------------------------------------------------------------------

float AltGPS;

void Gehps()
{
  bool newData = false;
  float flat, flon;
  unsigned long  age, date, t_ime, chars = 0;
  unsigned short sentences = 0, failed = 0;
  int year;
  byte month, day, hour, minute, second, hundredths;

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      if (gps.encode(c));    // encode gps data
      newData = true;
    }
  }

  Textzeilehm = String(hour) + ":" + String(minute);

  if (newData)
  {
    gps.f_get_position(&flat, &flon, &age);

    AltGPS = (gps.f_altitude());

    TextzeileG =                "" + String(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 5);
    TextzeileG = TextzeileG + "  " + String(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 5);
    TextzeileG = TextzeileG + "  " + String(AltGPS, 1); // + "m";

    //   TextzeileGd=String(year)+"/"+String(month)+"/"+String(day)+" "+String(hour)+":"+String(minute)+":"+String(second);
    //   TextzeileGd=TextzeileGd +"  "+String(int(gps.f_speed_kmph()))+"km"; //, TinyGPS::GPS_INVALID_F_SPEED;
    TextzeileGd = "" + String(hour) + ":" + String(minute) + ":" + String(second);  //UTC

    Serial.println(" ");
    Serial.println("GPS          " + TextzeileG);
    Serial.println(" ");
    Serial.println("GPSDatumZeit " + TextzeileGd);
    Serial.println(" ");
  }
}

//------end gps ---------------------------------------------------------------------------------



//----BME280-----------------------------------------------------------------------------
/*Connecting the BME280 Sensor:
  Sensor              ->  Board
  -----------------------------
  Vin (Voltage In)    ->  3.3V
  Gnd (Ground)        ->  Gnd
  SDA (Serial Data)   ->  SDA  D4
  SCK (Serial Clock)  ->  SCL  D3   0x76
  /**/

#include <EnvironmentCalculations.h>
#include <BME280I2C.h>

BME280I2C bme;

/*  BME280I2C::Settings settings(
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_Off );

    Default : forced mode, standby time = 1000 ms
    Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
  /**/
//--------------------------

float humi;     // für kompensationrechnung
float AltBaro;  // Höhe nach Barometer
float taupunkt;
//unsigned long bme_wartezeit;

void BME280Data()
{
  float temp(NAN), hum(NAN), pres(NAN);

  // if (bme_wartezeit < millis())
  // {

  TextzeileT = "";

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  BME280::Filter     filter(BME280::Filter_Off);
  BME280::Mode         mode(BME280::Mode_Forced);

  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

  bme.read(pres, temp, hum, tempUnit, presUnit);

  float altitude     = EnvironmentCalculations::Altitude(pres, envAltUnit);
  float dewPoint     = EnvironmentCalculations::DewPoint(temp, hum, envTempUnit);
  float seaLevelPres = EnvironmentCalculations::EquivalentSeaLevelPressure(altitude, temp, pres);

  taupunkt = dewPoint;
  AltBaro = altitude;
  TextzeileH = "BME alt Baro-GPS " + String(AltBaro - AltGPS) + "m ";

  Serial.println(TextzeileH);
  Serial.print("BME Höhe ü NN :  "); Serial.print(altitude);  Serial.print("m  Taupunkt: "); Serial.print(dewPoint); Serial.print("°C "); Serial.print("Luftdruck N.N.: ");  Serial.print(seaLevelPres / 100); Serial.println("hPa ");

  humi = hum / 100;
  if (humi > 0.99)
  {
    humi = 0.99; //für kompensationrechnung
  }

  TextzeileT = String(temp, 1) + "°C  " + String(hum, 1) + "%  " + String(seaLevelPres / 100, 1) + "hPa";

  //   bme_wartezeit = millis() + 2000;

  Serial.println("BME        " + TextzeileT);
  Serial.print  ("BME humi:             ");    Serial.println(humi);
  //Serial.println("BME presunit: " + String(presUnit) + " BME Modus: " + String(mode));

  //}   // end bme_wartezeit
}
//---end bme280 --------------------------------------------------------------------------------



//-CJMCU Anfang --------------------------------------------------------------------------------

/* Gedanken:

   RS = Sensorwiderstand beim messen
   R0 = ==> Rs0 Sensorwiderstand in frischer unbelasteter Luft

   erst in sauberer Luft Rs0 FESTSTELLEN dann kann gemessen werden

   RLoad = Schutzwiderstand in Reihe zum Sensor  hier  Nox 22000   Red 47000

   RS / R0 = besser RS / Rs0  =  Verhältnis der Widerstände

   Abgriffspannung am Messanschluß ist die Spannung die an Rload zu Masse abfällt

   Versorgungsspannung minus Abgriffspannung ist Sensorspannung Us

   Portexpander  2/3 gAIN = 0,1875 mV je Zähler  = 0,0001875 Volt je Zähler

   5Volt Board = max. Zähler 26666,67 bei 0,0001875
   eine andere Spannung verändert die maximale Zählerzahl



     CO
     --
     -  Messweite CO  999    ppm -> 1    - 1000 ppm
     -  Multiplitator ppm zu mg/m³  1,16412061629462
     -  Messweite CO in mg/m³    -> 1,16 - 1164 mg/m³
     -  Sensor ist für übliche CO Werte von 0,3 mg/m³ NICHT GEEIGNET --> müsste kleiner 0,4 ppm sein
        aber wohl für 8h-Grenzwert 10mg/m³
     -  Datenblatt R0 min 100.000  bis  max. 1.500.000  OHM
     -  lineares Kurvenbild senkrecht an y dann waagrecht an x Achse, parallel zu den Achsen
     -  doppellogarithmisches Kurvenbild sinkende Gerade
     -  Widerstandsverhältnis RS/R0 nimmt ab mit steigendem CO  --> Rs sinkt mit steigendem CO


     NO2
     ---
     -  Messweite NO2   9,95 ppm -> 0,05 -   10 ppm
     -  Multiplikator ppm zu µg/m³ 1912,02966843778
     -  Messweite NO2 in µg/m³   -> 95,6 - 19120,3 µg/m³
     -  Sensor ist für Fahrverbotsgrenzwert NOX 40 µg/m³ NICHT GEEIGNET  --> müsste kleiner 0,02 ppm sein
        aber wohl für 1h Grenzwert 200µg/m³ und Alarmschwelle 400µg/m³
     -  Datenblatt R0 min 800   bis   max. 20.000 OHM
     -  lineares Kurvenbild steigende gerade
     -  doppellogarithmisches Kurvenbild steigende Gerade
     -  Widerstandsverhältnis steigt mit steigendem NO2 -> RS steigt mit steigendem NO2
*/

/*
    Rückrechnung auf ppm
    http://myscope.net/auswertung-der-airpi-gas-sensoren/  Roland Ortner

   Gas-Sensor MiCS-2710 (MiCS-4514) für Stickstoffdioxid (NO2) wie  Sensor: MiCS-2710 (NOX)
   Stickstoffdioxid (NO2): 0.05 – 10ppm
   Rload: 22k
   Rs: 0.8 – 20 kOhm (Frischluft)
   var ppm = Math.pow(10, 0.9682 * (Math.log(Rs/R0) / Math.LN10) - 0.8108);
   NO2 ppm = 10^0,9682 *( log(Rs/R0) / ln10) - 0.8108)
   ----------------------------------------------------------------------------
   Gas-Sensor MiCS-5525 (MiCS-4514) für Kohlenstoffmonoxid (CO) wie  Sensor: MiCS-5525 (RED)
   Kohlenstoffmonoxid (CO): 1 – 1000ppm
   Rload0: 47k
   RS: 100k – 1500 kOhm (Frischluft)
   var ppm = Math.pow(10, -1.1859 * (Math.log(Rs/R0) / Math.LN10) + 0.6201);
   CO ppm = 10^-1.1859 * (log(Rs/R0) / ln10) + 0.6201)


  Umrechnung von ppm in mg/m3
  ---------------------------
    Die meisten Messgeräte und Sensoren liefern die Werte in ppm (parts per million),
    die Literatur oder Daten von Wetterdiensten geben hingegen meist die Werte in mg/m3 an

    ßi mg/m3 = (Mi g/mol * PRef mbar) / ( 10 * R J/K mol  * TRef K) * Xi ppm

    - ßi ist die Massenkonzentration des Gases in mg/m3
    - Mi ist die molare Masse der Komponente in g/mol
      --  Kohlenstoffmonoxid (CO): 28,01   g/mol
      --  Stickstoffdioxid  (NO2): 46,0055 g/mol
    - 10 Einheiten-Umrechnungsfaktor
    - R ist die Universelle Gaskonstante = 8,314472 J/K·mol
    - TRef Bezugstemperatur in K.  (Normtemperatur von 20 °C + 273,15 K = 293,15 K)
    - Xi ist der Messwert in ppm


    Umrechnung ppm in µg/m³
    -----------------------
    NO2 in µg/m3 = 1000 * ( 46,0055 *1013 ) / (10 * 8,314472 * 293,15) * NO2inPPM
    NO2 in µg/m3 = 1912,02966843778 * NO2inPPM
    (                  0.05  NO2 ppm =     95,6014834215  µg/m³   Sensor kann Grenze von 40 µg nicht messen
                       1     NO2 ppm =   1912,02966843778 µg/m³
                      10     NO2 ppm =  19120,2966843778  µg/m³
    )

    CO  in µg/m3 = 1000 * ( 28,01   *1013) / (10 * 8,314472 * 293,15) * COinPPM
    CO  in µg/m3 = 1164,12061629462 * COinPPM

    (                  1      CO ppm =    1164,12061629462 µg/m³  Sensor kann Werte von 200 µg (0,2mg) nicht messen
                    1000      CO ppm = 1164120,61629462    µg/m³
    )

    - Quellen: http://de.wikipedia.org, http://www.chemie.de/tools/

     Nachdem die Sensoren erst einige Tage lang „eingebrannt“ werden müssen,
     können sich die Werte zu Beginn ohne Grund ändern.

    SGX A1A-MiCS_AN2
    -- SGX Sensortech recommends placing the sensor behind a Teflon membrane
       in most applications. The Teflon membrane allows diffusion
       of the gases, while reducing the influence of the air speed.

    SGX SPF-2088
    -- stabilized Vo value is around 4.5V in these test conditions.
       To obtain the best resolution it is advised
       to work around the mid-operating range (i.e. Vcc/2 = 2.5V).
       To adjust Vo at 2.5V, load resistance has to be reduced.
       This is done by decreasing the resistance value of the potentiometer.
       This action has been done at time t0+100 min and is shown in the chart.


*/
//   ----------------------------------------------------------------------------


/*

  ( Spannungsteiler )
   U= R * I
   R= U / I
   I= U * R

  Sensorspannung RS = boardspannung-R0_Spannung

  Rs_no2= Us / I
        = (Ug - UR0) / R0 * UR0
        = Sensorspannung /  (R0_Spannung  * 22000)
        = (5 - (adc0 * 0,0001875 mV) / (adc0*0,0001875mV * 22000 OHM)

  Rs_co= Us / I
       =  Sensorspannung / (R0_Spannung  * 47000)
       =  (5 - (adc1 * 0,0001875 mV)) / (adc1*0,0001875mV * 47000 OHM)

  RS/R0 ->  Schaubild ppm  bzw Formel  Regressionsrechnung


  Spannungsteiler reihenschaltung   R1 / U1 = R2 / U2

    Rs / Us = R0 / U_R0
    RS = R0 / U_R0 * Us
    Rs = 22000 / ( adc0 * 0.0001875 ) * (5-(adc0 * 0.0001875))

    U  = R*I
    Rs = Us / I
    RS = Spannungsabfall rs / Strom I
       = (5v-     spannungsabfall R0) / (spannungsabfall Rload / Rload) ]

    Rs_no2 = ( ( 5000 - (adc0 * 0.0001875) ) / (( adc0 * 0.0001875))  / 22000);   // Rs ( U - (U/I))
     oder  =    22000 / ( adc0 * 0.0001875 ) * (5 - (adc0 * 0.0001875));          // r1/U1 = R2 /U2 -> R2= r1/u1*U2

  //float Rs_no2 = 22000 / ((adc0 * 0.0001875)) * (5 - (adc0 * 0.0001875)) ;
  //float Rs_co  = 47000 / ((adc1 * 0.0001875)) * (5 - (adc1 * 0.0001875)) ;


  Rload = 22k  / 47 k
  RS= R Sensor in Messsituation
  R0= R Sensor in sauberer Luft

    Spannungsteiler reihenschaltung   R1 / U1 = R2 / U2
    Reihenschaltung Strom ist gleich,  Spannung unterschiedlich

    Uload = Zaehler * Auflösung
    I = Uload / Rload

    URs = UBoard-Uload

    URs / Rs  = Uload / Rload     oder auch  Rs / URs  = RLoad / ULoad

    Rs= Rload / Uload * URs



  Regressionsrechnung nox nach ppm
  =======================
  f(x) = 0.993634072700759 x  - 0.800900894085055  Tabelle nach Ablesung

       = POTENZ(10;((0,993634072700759 * LOG10(RS/R0))      - 0,800900894085055)   // excel
  var ppm = pow(10,((0.993634072700759 * Math.log10(Rs/R0)) - 0.800900894085055);


  Regressionsrechnung co
  ======================
  f(x) = -1.17462679264927 x + 0.657906357391024

       = POTENZ(10;((-1,17462679264927  * LOG10(RS/R0))      + 0,657906357391024)  // excel
  var ppm = pow(10,((-1.17462679264927  * Math.log10(Rs/R0)) + 0.657906357391024);

*/
/*

  //double nmue = 1912.02966843778 *        (pow(10,  0.9682 * (log10(RsR0_no2) / log(10)) - 0.8108));  // NoX in µg/m³
  //double cmue = 1.16412061629462 * 1000 * (pow(10, -1.1859 * (log10(RsR0_co)  / log(10)) + 0.6201));  // co in µg/m³


  // versuch eigener formel aus schaubildablesung_3 basis trendlinienformel

  // NoX in µg/m³   [ umr auf µg   *  ppm ]
  float noxppm = (pow(10, 0.993634072700759 * (log10(RsR0_no2) ) - 0.800900894085055));
  float nmue   = 1912.02966843778  * noxppm;

  // co  in µg/m³   [ umr auf µg   *  ppm ]
  float coppm  = (pow(10, -1.17462679264927 * (log10(RsR0_co)  ) + 0.657906357391024));
  float cmue   = 1.16412061629462 * 1000 * coppm ;

*/



//---beginn CJMCU MiCS  NO2  CO  -------------------------------------------------------

// #include <math.h>      // für LN Berechnung no2 co  brauchts doch nicht

unsigned long  preheat;
int Vorheizzeit = 30000, beheizt = 0, z = 0, y = 0;
String PH = "";

/*----------- cjmcu -----------------------*/

void CJMCU()
{

  int16_t adc0, adc1, adc2, adc3;
  float   Noxadc0 = 0, Coadc1 = 0;

  if (beheizt < 1 )
  {
    if (y < 1) {
      digitalWrite(D7, HIGH); // CJMCU PREHEAT ein
      Serial.println();
      Serial.println("                                          preheat         ein");
      preheat = millis() + Vorheizzeit;
      PH = "P";                                             // P während preheat sonst N
      y = 1;
    }

    if ( millis() >= preheat ) {
      beheizt = 1;

      if (z < 1) {
        digitalWrite(D7, LOW); // CJMCU PREHEAT aus
        Serial.println("                                          preheat         aus");
        PH = "N";
        z = 1;
      }
    }
  }

  adc0 = ads.readADC_SingleEnded(0);  //NO2
  adc1 = ads.readADC_SingleEnded(1);  //CO
  adc2 = ads.readADC_SingleEnded(2);  //Boardspannung zum prüfen der Berechnung ??
  adc3 = ads.readADC_SingleEnded(3);  //MQ-135 ??

  Serial.print("adc0 "); Serial.print(adc0); Serial.print("  adc1 "); Serial.println(adc1);
  Serial.print("adc2 "); Serial.print(adc2); Serial.print("  adc3 "); Serial.println(adc3);

  TextzeileMiCS = "";
  TextzeileMiCSC = "";

  // Widerstand RS  R=U/I oder  Rs= Rload / Uload * URs
  // U=R*I    I=U/R    R=U/I
  // Widerstand Rs_ = Us/Is = Strom ist gleich in reihe  =  Urs    /   (Ur0/Rload)
  //                = (UBoard - Adceinstellung * zaehler)          /   (Adceinstellung * zaehler  / Rload)

  int Rload_no2 = 22000;
  int Rload_co  = 47000;
  int U_Board   = 5.0;                        // oder adc2*0.0001875  // Boardspannung

  float U_Rload_no2 = (adc0 * 0.0001875);
  float U_Rload_co  = (adc1 * 0.0001875);

  float I_no2 = (U_Rload_no2 / Rload_no2);
  float I_co  = (U_Rload_co  / Rload_co );

  float Rs_no2 = (U_Board - U_Rload_no2) / I_no2;
  float Rs_co  = (U_Board - U_Rload_co ) / I_co ;


  // Widerstandverhältnis RsR0_  =  Rs/R0 = Rs_  / R0

  float RsR0_no2 = Rs_no2 / 386158;            // 386158 aus Stachusmessung
  float RsR0_co  = Rs_co  / 9472;              //   9472 aus Stachusmessung

  //Textzeileppm = String(int(Rs_no2)) + "   " + String(int(Rs_co));

  Serial.println(" ");
  Serial.print("Rs_no2 Ohm ");
  Serial.println(Rs_no2);
  Serial.print("Rs_co  Ohm ");
  Serial.println(Rs_co);
  Serial.println(" ");

  Serial.println(" ");
  Serial.print("RsR0_no2 ");
  Serial.println(RsR0_no2);
  Serial.print("RsR0_co ");
  Serial.println(RsR0_co);
  Serial.println(" ");

  // hier formel für Kurve  aus myscope mit daten aus eigener Schaubild_3 ablesung

  // NoX in µg/m³   [ umr auf µg   *  ppm ]
  float noxppm = (pow(10, 0.993634072700759 * (log10(RsR0_no2) ) - 0.800900894085055));
  float nmue   = 1912.02966843778  * noxppm;

  // co  in µg/m³   [ umr auf µg   *  ppm ]
  float coppm  = (pow(10, -1.17462679264927 * (log10(RsR0_co)  ) + 0.657906357391024));
  float cmue   = 1.16412061629462 * 1000 * coppm ;

  Textzeileppm = "nppm " + String(noxppm, 3) + " cppm " + String(coppm, 3);

  Serial.println(" ");
  Serial.print("noxppm ");
  Serial.println(noxppm);
  Serial.print("coppm ");
  Serial.println(coppm);
  Serial.println(" ");

  Serial.println(" ");
  Serial.print("nmue ");
  Serial.println(nmue);
  Serial.print("cmue ");
  Serial.println(cmue);
  Serial.println(" ");

  TextzeileNOx = PH + " " + String(int(nmue));
  TextzeileCO =      "C " + String(cmue / 1000, 1);

  //TextzeileNOxCO = "RN " + String(int(Rs_no2)) + "  RC " + String(int(Rs_co));  // RS Widerstand

  Serial.println(" ");
  Serial.print("CJMCU  "); Serial.print(TextzeileNOx); Serial.print(" "); Serial.println(TextzeileCO);
  Serial.println(" ");

  Noxadc0 =  26666 - adc0;    // adc2 - adc0;

  //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  Coadc1  =  26666 - adc1;    // adc2 - adc1;

  // TextzeileMiCS=String("N "+String(Noxadc0)+"z C "+String(Coadc1)+"z");
  TextzeileMiCS  = String(PH + " " + String(int(Noxadc0)) + "");
  TextzeileMiCSC = String("C "     + String(int(Coadc1 )) + "");

  Serial.println(" ");
  Serial.println("CJMCU      Nox z    " + TextzeileMiCS);
  Serial.println("CJMCU      Red z    " + TextzeileMiCSC);
  Serial.println(" ");
}

//---Ende CJMCU MiCS NO2  CO        -------------------------------------------------------



//--SDS011  Anfang ------------------------------------------------------------------------

// Soll nur aufwachen, lesen, schlafen, 145 Sekunden warten

#include <SDS011_vers.h>
SDS011_vers my_sdsv;

float p10 = 0.0,   p25 = 0.0;
int sds_error,  zuerst = 0,   Pause = 145000;              // Pause sync zu airrohr 145000
unsigned long sds_wait = 0;

void Essds()
{
  if ((millis() >= sds_wait) or (zuerst < 1))
  {
    int sds_round = 0, _error = 0;
    do {
      _error = my_sdsv.SetWork();
      Serial.print("SDS Work _error:               "); Serial.println(_error);
      sds_round++;
      Serial.print("SDS wake round:                "); Serial.println(sds_round);
      if ( _error > 0 ) {
        delay(100);
      }
    }
    while ((_error not_eq 0) and (sds_round < 5));


    sds_round = 0;
    do {
      sds_error = my_sdsv.read_q(&p25, &p10);
      Serial.print("SDS read_q sds_error:          "); Serial.println(sds_error);
      sds_round++;
      Serial.print("SDS round:                     "); Serial.println(sds_round);
      if ( sds_error > 0) {
        delay(3000);
      }
    }
    while ((sds_error not_eq 0) and (sds_round < 5));

    s_err = sds_error;

    sds_round = 0;
    do {
      _error = my_sdsv.SetSleep();
      Serial.print("SDS sleep _error:              "); Serial.println(_error);
      sds_round++;
      Serial.print("SDS sleep round:               "); Serial.println(sds_round);
      if ( sds_error > 0) {
        delay(100);
      }
    }
    while ((_error not_eq 0) and (sds_round < 5));

    /**/


    /*
        my_sdsv.sleep();
        Serial.print("sleep ");
        delay(200);

        //________________

        int  queryset = 1; // 1 set Mode
        int  sleepwork = 1; // 0 sleep
        int  DevID1c = 0xFF;
        int  DevID2c = 0xFF;
        int  querysetr = 0;
        int  sleepworkr = 0;
        int  DevID1r = 0;
        int  DevID2r = 0;

        int error = my_sdsv.SetSleepAndWork(&queryset, &sleepwork, &DevID1c, &DevID2c, &querysetr, &sleepworkr, &DevID1r, &DevID2r);
        //delay(1000);
        Serial.print("SDS Work _error:                "); Serial.println(error);
        //error = 7;
        /**/ //________________

    /*
        //________________
        DevID1c = 0xFF;
        DevID2c = 0xFF;
        DevID1r = 0;
        DevID2r = 0;

        int sds_round = 0;
        do {
          sds_error = my_sdsv.QueryDataCommand(&DevID1c, &DevID2c, &p25, &p10, &DevID1r, &DevID2r);

          Serial.print("SDS_error:                      "); Serial.println(sds_error);
          Serial.print("SDS_Runde:                      "); Serial.println(sds_round);
          sds_round++;
          delay(3000);
        }
        while ((sds_error not_eq 0) and (sds_round < 5));

        Serial.print("SDS                        2,5: ");
        Serial.print(p25, 1); Serial.print("   10: "); Serial.println(p10, 1);

        Serial.print(DevID1r, HEX); Serial.print("   "); Serial.println(DevID2r, HEX);
        delay(900);
        /**/


    /*
        //________________
        queryset = 1; // 1 set Mode
        sleepwork = 0; // 0 sleep
        DevID1c = 0xFF;
        DevID2c = 0xFF;
        querysetr = 0;
        sleepworkr = 0;
        DevID1r = 0;
        DevID2r = 0;

        error = my_sdsv.SetSleepAndWork(&queryset, &sleepwork, &DevID1c, &DevID2c, &querysetr, &sleepworkr, &DevID1r, &DevID2r);
        Serial.print("SDS sleep _error:               "); Serial.println(error);
        Serial.println();
        /**/ //________________


    if (sds_error == 0)
    {
      TextzeileSDS = "2,5: " + String(p25, 0) + "   10: " + String(p10, 0) + "";
      Serial.println();
      //      Serial.print  ("SDS DeviceID: "); Serial.print(DevID1r); Serial.print(" "); Serial.println(DevID2r);
      Serial.println();
      Serial.println("SDS           " + TextzeileSDS);
      Essds_C();   // Feuchtekompensation aus gelesenen Werten
    }

    zuerst = 1;
    sds_wait = millis() + Pause;        // Zeit hochsetzen

  }
  //
  else {
    // my_sdsv.sleep();
    //Serial.print(" else sleep ");
  }
}
//---ende sds--------------------------------------------------------------------------------



//---beginn Kompensationsrechnung für sds-------------------------------------------------------

/*
  aus http://www.opengeiger.de/Feinstaub/FeuchteKompensation.pdf

  ...   Growth-Factor   GF(RH) = a + (b*RH^2) / (1-RH)

  Darin sind a und b empirisch bestimmte Parameter für die Korrektur.
  In dem hier beschriebenen Beispiel wurden die Werte:
  a=1  und b=0.25 aus den genannten Publikationen übernommen.
  Damit  kann nun der pro  Messung  bestimmte Growth-Factor
  zur  Kompensation der einzelnen Laser-Streulicht PM-Werte benutzt werden
  um die gravimetrischen PM-Werte zu approximieren:

  Gravimetrischer PM-Schätzwert = Streulicht-PM-Wert / GF(RH)
  also kompensierter PM = gelesener PM / (1+(0.25*Feuchte^2) / (1-Feuchte) )

  /**/
//---ende   Kompensationsrechnung für sds-------------------------------------------------------



//--SDS011_C-------------------------------------------------------------------------
float p25c, p10c;

void Essds_C()
{

  // humi z.B. 0.49 für 49% Feuchte
  /*
    // sds011 Kompensierung nach opengeiger.de  dämpft stärker als piotrkpaul (hackair)
    p25c = (p25 / (1 + (0.25 * humi * humi) / (1 - humi)));
    p10c = (p10 / (1 + (0.25 * humi * humi) / (1 - humi)));
    /**/

  /*  sds011 kompensieren nach piotrkpaul hackair  nicht so stark wie opengeiger
      https://github.com/piotrkpaul/esp8266-sds011/blob/master/sds011_nodemcu/sds011_nodemcu.ino
    /**/

  p25c = (p25 / (1 + (0.48756 * pow(humi, 8.60068)))); //kompensation piotrkpaul aus hackair
  p10c = (p10 / (1 + (0.81559 * pow(humi, 5.83411))));
  /**/

  TextzeileSDSc = "25: " + String(p25c, 1) + " 10: " + String(p10c, 1) + "";
  Serial.println("SDSc       " + TextzeileSDSc);
}
//----Ende SDS_C---------------------------------------------------------------



//-----------------------------------------------------------------------------
void Texte()
{
  /*
    123456789012345678901234!567
    !-----------------------!---!
    N 2,023 µ  C 9794 µ
    P25  5  P10  17
    P25c 4  P10c 12
    48,214  11,557  528m
    2018/03/25  21:52:31  3km
    26 °C 35 %  941 hPA
    - - - -- - - -- -
    N 2,023 µ  C 9794 µ
    P25c 4  P10c 12
    2018/03/25  21:52:31  3km
    26°C 35%  941 hPA
    - - - -- - - -- -
    N 2,023 µ  UTC: 10:10:10
    C 9794 µ            144s
    P25= 4 i= 3   P10= 12
    48,214  11,557  528m
    26°C 35%  941hP
    - - - -- - - -- -
    /**/

  // Display 128*64 Pixel 0-127 0-63 /  6 Zeilen ( 1*16hoch Orange  +  4*10hoch=56 Rest 8)
  // oder  4 Zeile a 16 hoch =64
  // oder  8 Zeile a  8 Pixel hoch

  // 6 Zeilen 16&10 1: 16 0,0  zeile 2-6: 10 0, 14 24 34 44 54
  // 4 Zeilen 16     : 16 0, 0,16,32,48

  // T emperatur, G ps, Gps_d atum, SDS, SDSc, MiCS

  sek = s_err + " " + String(int((sds_wait - millis()) / 1000)) + "";
  Serial.println("              Sekunden bis SDS: " + sek);
  Serial.println();

  /**/

  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);

  display.drawString(128, 10, TextzeileG);      // Blau gps Lat Lon höhe
  display.drawString(128, 20, String(AltBaro, 1));  // Blau Höhe aus Barometer


  display.setTextAlignment(TEXT_ALIGN_CENTER);

  display.drawString(64, 20, sek);
  //display.drawString(64, 20, Textzeileppm);
  //display.drawString(64, 32, TextzeileNOxCO);   // no2 co rs/r0

  display.setTextAlignment(TEXT_ALIGN_LEFT);

  display.drawString(00, 00, TextzeileT);      // Blau C % hPa Temp Feuchte Luftdruck
  // display.drawString(00, 12, TextzeileSDS); // Blau Pm25  PM10
  // display.drawString(00, 10, TextzeileG);      // Blau gps Lat Lon höhe
  display.drawString(00, 20, Textzeilehm);     // GPS UTC hh:min

  display.setFont(ArialMT_Plain_16);

  display.drawString(00, 32, TextzeileSDS);    // Blau PM25c PM10c

  //display.drawString(00, 32, Textzeileppm);
  display.drawString(00, 48, TextzeileNOx);    // no2  Werte
  //display.drawString(00, 48, TextzeileMiCS);   // Orange No2
  //display.drawString(64, 48, TextzeileMiCSC);  // Orange CO

  display.setTextAlignment(TEXT_ALIGN_RIGHT);

  display.drawString(128, 48, TextzeileCO);    //  co Werte
}
//--End Texte -------------------------------------------------------------



//-----Begin Setup ----------------------------------------------------------------------

void setup() {
  //--------------------------------------------

  Serial.begin(9600);                 // für Ausgabe im seriellen Monitor

  /*---------------------------------------------------------*/

  ss.begin(9600);                  // SoftSerial für GPS

  /*--------------------------------------------------------*/

  Wire.begin(D4, D3);                 // D4,D3   <--I2c-Kanäle SDA, SCL

  /*--------------------------------------------------------*/

  display.init();                     // Initialising the UI will init the display too.
  // display.flipScreenVertically();  // nicht kippen, damit bei einbaurichtung sds aufstellen ermöglicht wird.
  display.setFont(ArialMT_Plain_10);  // <-------------------------------- Font Set
  display.clear();                    // clear the display

  /*--------------------------------------------------------*/

  pinMode(LED_BUILTIN, OUTPUT);       // interne LED als Output definieren
  digitalWrite(LED_BUILTIN, HIGH);    // Ausschalten  /**/
  /*--------------------------------------------------------*/

  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);                // WLAN ausschalten  /**/
  //--------------------------------------------

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads.begin();
  //                               //ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)
  ads.setGain(GAIN_TWOTHIRDS);     // 2/3x gain +/- 6.144V  1 bit = 3mV          0.1875mV (default)

  /*--------------------------------------------------------*/

  bme.begin();                     //Starte bme280

  /*--------------------------------------------------------*/

  my_sdsv.begin(D1, D2);   //  my_sds.begin(RX-ESP(TX-SDS),TX-ESP(RX-SDS));


  int _error = my_sdsv.SetQueryReportingMode();
  Serial.print(" Setup SetQueryReportingMode _error:  "); Serial.println(_error);
  _error = my_sdsv.SetContinuousMode();
  Serial.print(" Setup SetContinuousMode _error:      "); Serial.println(_error);
  _error = my_sdsv.SetSleep();
  Serial.print(" Setup SDS sleep _error:              "); Serial.println(_error);
  /**/


  /*
    int queryset = 1;    // 1 set Mode
    int activequery = 1;  // 0 activereport Mode (DEFAULT)  1 querymode
    int DevID1c = 0xFF;  // FF= all
    int DevID2c = 0xFF;  // FF= all

    int querysetr ;
    int activequeryr;
    int DevID1r;
    int DevID2r;

    int  error = my_sdsv.SetDataReportingMode(&queryset, &activequery, &DevID1c, &DevID2c, &querysetr, &activequeryr, &DevID1r, &DevID2r);
    delay(500);
    /**/

  /*
    queryset = 1;    // 1 set Mode
    int contperiod = 0; // 0 Continuous Mode (DEFAULT), Working Period 1-30 min (incl 30 sec Measurement)
    DevID1c = 0xFF;  // FF= all
    DevID2c = 0xFF;  // FF= all
    querysetr = 3;
    int contperiodr = 3;
    DevID1r = 3;
    DevID2r = 3;

    error = my_sdsv.SetWorkingPeriod(&queryset, &contperiod, &DevID1c, &DevID2c, &querysetr, &contperiodr, &DevID1r, &DevID2r);
    delay(500);
    /**/

  /*
    queryset = 1; // 1 set Mode
    int  sleepwork = 0; // 0 sleep
    DevID1c = 0xFF;
    DevID2c = 0xFF;
    querysetr = 0;
    int  sleepworkr = 0;
    DevID1r = 0;
    DevID2r = 0;

    error = my_sdsv.SetSleepAndWork(&queryset, &sleepwork, &DevID1c, &DevID2c, &querysetr, &sleepworkr, &DevID1r, &DevID2r);
    /**/
  /*--------------------------------------------------------*/

  yield();
}
//------------= ENDE Setup ------------------------------------------------------





//---LOOP------------------------------------------------------------------------
void loop()
{
  display.clear();

  BME280Data();

  Gehps();

  CJMCU();

  Essds();               // --> ESSDSc

  Texte();

  display.display();     // write the buffer to the display

  yield();
}
//---LOOP ENDE------------------------------------------------------------------
//------------------------------------------------------------------------------

