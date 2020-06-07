// Original code by Rolf R Bakke, Oct 2012
// http://wildlab.org/index.php/2015/07/07/arduino-variometer/
// https://www.rcgroups.com/forums/showthread.php?1749208-DIY-simple-and-inexpensive-Arduino-based-sailplane-variometer
//
// ==================================================
// Adapté par Xavier Chavanet pour Heltec LoRa 32 v1
// ==================================================
//
// Hardware:
//     Heltec LoRa : https://www.amazon.fr/gp/product/B078LXL5ZK/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1
//     Capteur MS5611 (i2c) : https://www.amazon.fr/gp/product/B07J2S5RJV/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&psc=1
//
//     RF 433 solution (for ogiginal code)
//          https://www.rfsolutions.co.uk/radio-modules-c10/am-rt14-433p-low-cost-am-hybrid-transmitter-module-p344
//          https://www.rfsolutions.co.uk/radio-modules-c10/am-superhet-receiver-433-92mhz-sil-module-euro-pinout-p658
//
// Variation pression 
//    1hPa 28ft 8,53m soit 11,7 pa /m
//
#define debug false
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <SSD1306.h>

#include <LoRa.h>

const byte led = 25;
const byte buttonPin = 0;

unsigned int calibrationData[7];
unsigned long milliTime = 0;

float deltaPression = 0;
float deltaTime = 0;

char str[132];

float elevation1s = 0;
unsigned long savedTime1s;
float savedPression1s = 0;

float elevationxs = 0;
unsigned long savedTimexs;
float savedPressionxs = 0;

#define integrationVario 3000 // en millisecondes

float toneFreq, toneFreqLowpass, pressure, lowpassFast, lowpassSlow ;

int ddsAcc;

// WIFI_LoRa_32 ports
//--------------------------------------
// GPIO5 — SX1278’s SCK
// GPIO19 — SX1278’s MISO
// GPIO27 — SX1278’s MOSI
// GPIO18 — SX1278’s CS
// GPIO14 — SX1278’s RESET
// GPIO26 — SX1278’s IRQ(Interrupt Request)
#define SCK 5
#define CS 18
#define RST 14
#define MISO 19
#define MOSI 27
#define DI0 26

#define BAND 469E6 //433E6// 902E6 //915E6

// I2C ports
#define SDA 4
#define SCL 15
#define screenResetPin 16

//
// Les capteurs
//
SSD1306 display(0x3c, SDA, SCL);

//===============================================================
// setup
//=============================================================== 
void setup()
{
  Wire.begin();
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  pinMode(buttonPin, INPUT);
  // ---------------------------------------------------------------------
  // Initialising the UI will init the display too.
  // ---------------------------------------------------------------------
  pinMode(screenResetPin, OUTPUT);
  digitalWrite(screenResetPin, LOW); // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(screenResetPin, HIGH);
  
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5, 5, "Vario ligne treuil");
  display.drawString(5, 20, "Version: ");
  display.drawString(5, 30, String(__DATE__) + " " + String(__TIME__) );
  display.display();
  
  // ---------------------------------------------------------------------
  // Setup MS5611
  // ---------------------------------------------------------------------
  setupSensor();
  pressure = getPressure();
  lowpassFast = lowpassSlow = savedPression1s = savedPressionxs = pressure;
  savedTime1s = savedTimexs = millis();
   
// ---------------------------------------------------------------------
// Initialisation LoRa
// ---------------------------------------------------------------------
  //SPI.begin(5, 19, 27, 18);
  SPI.begin(SCK, MISO, MOSI, CS);
  
  LoRa.setPins(SS, RST, DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.enableCrc();
  LoRa.setSyncWord(0xF2); 

  display.drawString(5, 45, "Initialisations ok" );
  display.display();
  delay (1000);
  display.clear();
  display.display();
}

//===============================================================
// loop
//=============================================================== 

void loop()
{
  // Lecture pin 0 (bouton PRG) Si set, alors simulation 
  if (digitalRead(buttonPin) == LOW) {
      delay (1000);
      for (float i=0; i<10; i++){
          float f1 = i * 0.6 ;
          sendLora(f1,f1,1015);
          delay (1000);
      }
  }  

  // Lecture sur ms5611
  //---------------------
  pressure = (float)getPressure();

  // Filtrage du signal
  //--------------------
  lowpassFast = lowpassFast + (pressure - lowpassFast) * 0.1;
  lowpassSlow = lowpassSlow + (pressure - lowpassSlow) * 0.05;

  //Serial.print(lowpassFast);
  //Serial.print(',');
  //Serial.print(lowpassSlow);
  //Serial.print(',');
  //Serial.println(pressure);
  
// Taux de monté sur une seconde
//---------------------------------
  if (savedTime1s + 1000 < millis() ) {
      deltaPression = savedPression1s - lowpassFast;
      deltaTime = (millis() - savedTime1s);
      elevation1s = ( deltaPression / 11.7 ) / (deltaTime / 1000) ;
      savedTime1s = millis();
      savedPression1s = lowpassFast;
      
      // Envoi LoRa
      sendLora(elevation1s, elevationxs,savedPression1s);

}

// Taux de montée sur integrationVario millisecondes
//--------------------------------------------------
  if (savedTimexs + integrationVario < millis() ) {
      deltaPression = savedPressionxs - lowpassFast;
      deltaTime = (millis() - savedTimexs);
      elevationxs = ( deltaPression / 11.7 ) / (deltaTime / 1000);
      savedTimexs = millis();
      savedPressionxs = lowpassFast;
}

// Affichage des taux de montée  displayData ();
   // displayData(); Pas affiché temps de traitement +20%

  // Boucle capée à 20ms
  //
  while (millis() < milliTime);        //loop frequency timer
  milliTime += 20;
}


long getPressure()
{
  long D1, D2, dT, P;
  float TEMP;
  int64_t OFF, SENS;
 
  D1 = getData(0x48, 10);
  D2 = getData(0x50, 1);

  dT = D2 - ((long)calibrationData[5] << 8);
  TEMP = (2000 + (((int64_t)dT * (int64_t)calibrationData[6]) >> 23)) / (float)100;
  OFF = ((unsigned long)calibrationData[2] << 16) + (((int64_t)calibrationData[4] * dT) >> 7);
  SENS = ((unsigned long)calibrationData[1] << 15) + (((int64_t)calibrationData[3] * dT) >> 8);
  P = (((D1 * SENS) >> 21) - OFF) >> 15;
  
  //Serial.println(TEMP);
  //Serial.println(P);
  
  return P;
}


long getData(byte command, byte del)
{
  long result = 0;
  twiSendCommand(0x77, command);
  delay(del);
  twiSendCommand(0x77, 0x00);
  Wire.requestFrom(0x77, 3);
  if(Wire.available()!=3) Serial.println("Error: raw data not available");
  for (int i = 0; i <= 2; i++)
  {
    result = (result<<8) | Wire.read(); 
  }
  return result;
}


void setupSensor()
{
  twiSendCommand(0x77, 0x1e);
  delay(100);
  
  for (byte i = 1; i <=6; i++)
  {
    unsigned int low, high;

    twiSendCommand(0x77, 0xa0 + i * 2);
    Wire.requestFrom(0x77, 2);
    if(Wire.available()!=2) Serial.println("Error: calibration data not available");
    high = Wire.read();
    low = Wire.read();
    calibrationData[i] = high<<8 | low;
    Serial.print("calibration data #");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println( calibrationData[i] ); 
  }
}


void twiSendCommand(byte address, byte command)
{
  Wire.beginTransmission(address);
  if (!Wire.write(command)) Serial.println("Error: write()");
  if (Wire.endTransmission()) 
  {
    Serial.print("Error when sending command: ");
    Serial.println(command, HEX);
  }
}

void ledOn()
{
  digitalWrite(led,1);
}


void ledOff()
{
  digitalWrite(led,0);
}
//========================================================
// Display data
//========================================================
void displayData () {
// Display  
  display.clear();
  display.setFont(ArialMT_Plain_10);
  dtostrf(elevation1s,2,1,str);
  display.drawString(5, 5, "Instantané : " + String(str));

  display.drawString(5, 25, "Intégré : ");
  display.setFont(ArialMT_Plain_24);
  dtostrf(elevationxs,2,1,str);
  //display.drawString(10, 45, String(str));

  display.display();  
 }
//==================================================================
// Send LoRa
//=================================================================
 void sendLora (float tx1s, float txxs, float pression){

  // Trame envoyée  :
  //vario;elevation1s;elevationxs;pression
 //Serial.println (pression);      
//  if (tx1s > 0.2) {
      //ledOn();
      
      // Envoi des données sur LORA
      LoRa.beginPacket();
      
      // entete
      LoRa.print("vario;");
      
      // taux de monté sur 1 seconde
      dtostrf(tx1s,2,1,str);
      LoRa.print(String(str)  + ";");

      // taux de monté sur integrationVario millisecondes
      dtostrf(txxs,2,1,str);
      LoRa.print( String(str) + ";");
      
      // Pression locale
      dtostrf(pression,2,1,str);
      LoRa.print(String(str)  + ";");   
         
      LoRa.endPacket();
     
      //ledOff(); 
 // }
 }
