#include <Wire.h>
#include <SPI.h>

///////////Nuestro TRANSMISION
#include <RH_RF95.h>
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

#define RF95_FREQ 915.0
//test
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


///////////////////////////////////////////////////////////////////////IMU
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SparkFunLSM9DS1.h>
LSM9DS1 imu;

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW


#define PRINT_CALCULATED
#define PRINT_SPEED 250 // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.


//////////////////////////////////////////////////////////////////////BMP (Barometro, temp int, alt) (MORADO) (SPI)
#include <Adafruit_BMP280.h>
#define BMP_CS A2 //CSB
#define BMP_SCK A3 //SCL
#define BMP_MISO A4  //SDO
#define BMP_MOSI A5   //SDA
float Temp; 
float Pres;
float Alt;
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);

///////////////////////////////TempHum
#include <TH02_dev.h>
#include "Arduino.h"
#include "Wire.h" 

////////////////////////////////////////////////GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 9);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean);


void setup() 
{
   Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }
  
//////////////////////////////////////////////////////////////////GPS
 Serial.println("Adafruit GPS library basic test!");
  GPS.begin(115200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  //delay(1000);
  // Ask for firmware version
 mySerial.println(PMTK_Q_RELEASE);
  
///////////////////////////////////////////////////////////////////IMU
imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin())
  {
    //Serial.println("Failed to communicate with LSM9DS1.");
  //  Serial.println("Double-check wiring.");
   // Serial.println("Default settings in this sketch will " \
   //               "work for an out of the box LSM9DS1 " \
   //               "Breakout, but may need to be modified " \
    //              "if the board jumpers are.");
    while (1)
      ;
  }
  

//////////////////////////////////////////////////////////////////TempHum
//Serial.println("****TH02_dev demo by seeed studio****\n");
  // Power up,delay 150ms,until voltage is stable 
  delay(150);
  // Reset HP20x_dev 
  TH02.begin();
  delay(100);
  
  // Determine TH02_dev is available or not 
 // Serial.println("TH02_dev is available.\n");   
  
  ///////////////////////////////////////////////////////////////////////////////////////BMP
//Serial.println(F("BMP280 test"));

  while (!bmp.begin()) {
    //Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    ;
 }
    //Default settings from datasheet. 
 bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode. 
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling 
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling 
                   Adafruit_BMP280::FILTER_X16,      // Filtering. 
                   Adafruit_BMP280::STANDBY_MS_500); // Standby time. 
  /////////////////////////////////////////////////////////////////////////////////////////Comunicacion
 pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  //delay(100); 
  
  //Serial.println("Feather LoRa TX Test!");
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
  //  Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    //Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);  
}




/////////////////////////////////////////////////////////////////ETC DE GPS
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();



void loop()
{
//////////////////////////////////////////////////////GPS
if (! usingInterrupt) {
   
    char c = GPS.read();
   
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  if (GPS.newNMEAreceived()) {
  
    if (!GPS.parse(GPS.lastNMEA()))   
      return;  
  }

  if (timer > millis())  timer = millis();

 /* if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    if (GPS.fix) {
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 20);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 20);
      
      Serial.print("Altitude: "); Serial.println(GPS.altitude); 
    }
  }*/
////////////////////////////////////////////////////////////Transmision GPS
/*float hora;
float minuto;
float sec;
float milisec;
float dia;
float mes;
float year;*/
float latitud;
//float longitud;
float altgps;

    /*  hora=GPS.hour;
      minuto= GPS.minute;
      sec=GPS.seconds;
      milisec=GPS.milliseconds ;
      dia=GPS.day, DEC;
      mes=GPS.month, DEC;
      year= GPS.year, DEC;*/
      latitud= GPS.latitudeDegrees;
      altgps= GPS.longitudeDegrees;

/* //////////////////////////hora
 char HORA[20];
 //Serial.print("Sending ="); Serial.print(hora); Serial.println(" Horas");
 dtostrf(hora, 6, 2, HORA);
 rf95.send((uint8_t *)HORA, 20);
 delay(227);      
 ///////////////////////////minuto
 char MINUTO[20];
 //Serial.print("Sending ="); Serial.print(minuto); Serial.println(" Minuto");
 dtostrf(minuto, 6, 2, MINUTO);
 rf95.send((uint8_t *)MINUTO, 20);
 delay(227);      
 ///////////////////////////sec
 char SEC[20];
 //Serial.print("Sending ="); Serial.print(sec); Serial.println(" Segundos");
 dtostrf(sec, 6, 2, SEC);
 rf95.send((uint8_t *)SEC, 20);
 delay(227);      
  //////////////////////////milisec
 char MILISEC[20];
 //Serial.print("Sending ="); Serial.print(milisec); Serial.println(" Milisegundos");
 dtostrf(milisec, 6, 2, MILISEC);
 rf95.send((uint8_t *)MILISEC, 20);
 delay(227);
  //////////////////////////dia
 char DIA[20];
 //Serial.print("Sending ="); Serial.print(dia); Serial.println(" Dia");
 dtostrf(dia, 6, 2, DIA);
 rf95.send((uint8_t *)DIA, 20);
 delay(227);     
  /////////////////////////MES
 char MES[20];
 //Serial.print("Sending ="); Serial.print(mes); Serial.println(" Mes");
 dtostrf(mes, 6, 2, MES);
 rf95.send((uint8_t *)MES, 20);
 delay(227); 
  //////////////////////////YEAR
 char YEAR[20];
 //Serial.print("Sending ="); Serial.print(year); Serial.println(" Año");
 dtostrf(year, 6, 2, YEAR);
 rf95.send((uint8_t *)YEAR, 20);
 delay(227);*/
 /////////////////////////////Latitud
 char LATITUD[20];
 //Serial.print("Sending ="); Serial.print(latitud); Serial.println(" Latitud");
 dtostrf(latitud, 6, 10, LATITUD);
 rf95.send((uint8_t *)LATITUD, 20);
 delay(227);
  /////////////////////////Altitud
 char ALTGPS[20];
 //Serial.print("Sending ="); Serial.print(altgps); Serial.println(" Altitud de GPS");
 dtostrf(altgps, 6, 10, ALTGPS);
 rf95.send((uint8_t *)ALTGPS, 20);
 delay(227);
 
  ///////////////////////////////////////////////////IMU
//  if ( imu.gyroAvailable() )
 // {
    imu.readGyro();
// }
//  if ( imu.accelAvailable() )
 // {
    imu.readAccel();
 // }
 // if ( imu.magAvailable() )
//  {
    imu.readMag();
 // }
  
  if ((lastPrint + PRINT_SPEED) < millis())
  {
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    /*printAttitude(imu.ax, imu.ay, imu.az, 
                 -imu.my, -imu.mx, imu.mz);*/
    //Serial.println();
    
    lastPrint = millis(); // Update lastPrint time
  }

 //////////////////////////////////////////////TempHum 
float temper = TH02.ReadTemperature(); 
  /* Serial.println("Temperature: ");   
   Serial.print(temper);
   Serial.println("C\r\n");*/
   
   float humidity = TH02.ReadHumidity();
   /*Serial.println("Humidity: ");
   Serial.print(humidity);
   Serial.println("%\r\n");*/
  // delay(1000);

   //////////////////////Transmision TempHum 
  char TEMPER[20];
   //Serial.print("Sending = "); Serial.print(temper);Serial.println("*C");
   dtostrf(temper, 6, 2, TEMPER);
   rf95.send((uint8_t *)TEMPER, 20);
   delay(227);

   char HUM[20];
   //Serial.print("Sending = "); Serial.print(humidity);Serial.println("%");
   dtostrf(humidity, 6, 2, HUM);
   rf95.send((uint8_t *)HUM, 20);
   delay(227);


  ////////////////////////////////////////////////////////////////////////////////////////BMP
//Serial.print(F("Temperature = "));
    Temp = bmp.readTemperature();
    //Serial.print(Temp);
    //Serial.println(" *C");

    //Serial.print(F("Pressure = "));
    Pres = bmp.readPressure();
    //Serial.print(Pres);
    //Serial.println(" Pa");

    // Serial.print(F("Approx altitude = "));
     Alt = bmp.readAltitude(); // Adjusted to local forecast! 
     //Serial.print(Alt);
     //Serial.println(" m");

    //Serial.println();
   //delay(1);//2000
   
 ////////////////////////////////////////////////////////////////////////////////////////Transmision BMP
 
 //delay(1);//3000
  //Serial.println("Transmitting..."); // Send a message to rf95_server
 // delay(227);
  
  //Temperatura
  char Temperatura[8];
  //Serial.print("Sending = "); Serial.print(Temp);Serial.println("*C");
  dtostrf(Temp, 6, 2, Temperatura);
  rf95.send((uint8_t *)Temperatura, 8);
  delay(227);
  
  //Presion
  char Presion[12];
  //Serial.print("Sending = "); Serial.print(Pres);Serial.println("Pa");
  dtostrf(Pres, 6, 2, Presion);
  rf95.send((uint8_t *)Presion, 12);
  delay(227);
  
  //Altitud
  char Altitud[8];
  //Serial.print("Sending = "); Serial.print(Alt);Serial.println("m");
  dtostrf(Alt, 6, 2, Altitud);
  rf95.send((uint8_t *)Altitud, 8);
  delay(227);
  Serial.println("VUELTA");
  delay(1500);
Serial.flush();


  
  //////////////////////////////////////////////////////////////////////////////////Comunicacion
  rf95.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
 // delay(227);
  
  //Serial.println("Waiting for reply...");
  if (rf95.waitAvailableTimeout(3000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
     // Serial.print("Got reply: ");
      Serial.println((char*)buf);
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      //Serial.println("Receive failed");
    }
  }
  else
  {
   // Serial.println("No reply, is there a listener around?");
  }
}


/////////////////////////////////////////////////VOIDS DE IMU
void printGyro()
{
  //Serial.print("G: ");
#ifdef PRINT_CALCULATED
 /* Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");*/
  float gyrox;
  float gyroy;
  float gyroz;
         gyrox=imu.calcGyro(imu.gx);
         gyroy=imu.calcGyro(imu.gy);
         gyroz=imu.calcGyro(imu.gz);

         /////////////gyroX
         char GYROX[20];
        /* Serial.print("Sending ="); 
         Serial.print(gyrox);
         Serial.println(" grados X");*/
         dtostrf(gyrox, 6, 2, GYROX);
         rf95.send((uint8_t *)GYROX, 20);
         delay(227);
          /////////////gyroY
         char GYROY[20];
         //Serial.print("Sending ="); Serial.print(gyroy);Serial.println(" grados Y");
         dtostrf(gyroy, 6, 2, GYROY);
         rf95.send((uint8_t *)GYROY, 20);
         delay(227);
           /////////////gyroZ
         char GYROZ[20];
       //  Serial.print("Sending ="); Serial.print(gyroz);Serial.println(" grados Z");
         dtostrf(gyroz, 6, 2, GYROZ);
         rf95.send((uint8_t *)GYROZ, 20);
         delay(227);
         
  
  
#elif defined PRINT_RAW
 /* Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);*/
#endif
}

void printAccel()
{  
  //Serial.print("A: ");
#ifdef PRINT_CALCULATED
  /*Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");*/
 float Accelx;
  float Accely;
  float Accelz;
         Accelx=imu.calcAccel(imu.ax);
         Accely=imu.calcAccel(imu.ay);
         Accelz=imu.calcAccel(imu.az);

         /////////////AccelX
         char ACCELX[20];
        /* Serial.print("Sending ="); 
         Serial.print(Accelx);
         Serial.println(" g X");*/
         dtostrf(Accelx, 6, 2, ACCELX);
         rf95.send((uint8_t *)ACCELX, 20);
         delay(227);
          /////////////AccelY
         char ACCELY[20];
         //Serial.print("Sending ="); Serial.print(Accely);Serial.println(" g Y");
         dtostrf(Accely, 6, 2, ACCELY);
         rf95.send((uint8_t *)ACCELY, 20);
         delay(227);
           /////////////AccelZ
         char ACCELZ[20];
         //Serial.print("Sending ="); Serial.print(Accelz);Serial.println(" g Z");
         dtostrf(Accelz, 6, 2, ACCELZ);
         rf95.send((uint8_t *)ACCELZ, 20);
         delay(227);
         
         
#elif defined PRINT_RAW 
 /* Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);*/
#endif

}

void printMag()
{  
  //Serial.print("M: ");
#ifdef PRINT_CALCULATED
 /* Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");*/
  float Magx;
  float Magy;
  float Magz;
         Magx=imu.calcMag(imu.mx);
         Magy=imu.calcMag(imu.my);
         Magz=imu.calcMag(imu.mz);

         /////////////MagX
         char MAGX[20];
        /* Serial.print("Sending ="); 
         Serial.print(Magx);
         Serial.println(" gauss X");*/
         dtostrf(Magx, 6, 2, MAGX);
        // rf95.send((uint8_t *)MAGX, 20);
         //delay(227);
          ////////////MagY
         char MAGY[20];
        // Serial.print("Sending ="); Serial.print(Magy);Serial.println(" gauss Y");
         dtostrf(Magy, 6, 2, MAGY);
         rf95.send((uint8_t *)MAGY, 20);
        delay(227);
           /////////////MagZ
         char MAGZ[20];
        // Serial.print("Sending ="); Serial.print(Magz);Serial.println(" gauss Z");
         dtostrf(Magz, 6, 2, MAGZ);
         rf95.send((uint8_t *)MAGZ, 20);
         delay(227);
         
      
#elif defined PRINT_RAW
  /*Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);*/
#endif
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0)
    heading = (mx < 0) ? PI : 0;
  else
    heading = atan2(mx, my);
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
 /* Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);*/
}
