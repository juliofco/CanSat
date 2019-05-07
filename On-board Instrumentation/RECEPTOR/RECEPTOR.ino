#include <Adafruit_BMP280.h>
//#include <TinyWireM.h>
//#include <USI_TWI_Master.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
 
// nuestro sensor
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
//IMU
#define BMP_CS A2
#define BMP_SCK A3
#define BMP_MISO A4
#define BMP_MOSI A5 

Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

// Frecuencia debe coincidir con elemisor 915.0 o 434.0
#define RF95_FREQ 915.0
 
// Instancia
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
// Blinky on receipt
#define LED 13
 
void setup()
{
/////////////////////////////////////////////////////////////////////////////////////////IMU
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));

 //while (!bmp.begin()) {
   //Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  // }
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  ////////////////////////////////////////////////////////////////////////////////////////
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }
  //delay(100);
 
  Serial.println("Feather LoRa RX Test!");
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
}
 
void loop()
{
///////////////////////////////////////////////////////////////////////////////////////////////IMU
/*Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
   
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude()); /* Adjusted to local forecast! 
    Serial.println(" m");

    Serial.println();
    delay(2000);*/
///////////////////////////////////////////////////////////////////////////////////////////////
  if (rf95.available())
  {
    // Nos llego algo??
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
 
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      //Nombre de lo que nos dan
      Serial.print("Got: ");
      //Dato que queremos
      Serial.println((char*)buf);
      //Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
      
      // Respuesta
    /*  int boton;   ///////////////////////////////////////////////VER QUE PEZ CON LO DE GUI
      boton=     //////Poner valor de boton del gui
      if(boton>0)
      {
         rf95.send((uint8_t *)boton, 8);  
      }*/
      //uint8_t data[] = "And hello back to you";
      //rf95.send(data, sizeof(data));
     //rf95.waitPacketSent();
      //Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
             
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}
