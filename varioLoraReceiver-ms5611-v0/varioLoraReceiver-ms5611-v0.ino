/* 
 *  varioLoraReceiver
 *  =================
 *  Prévu pour fonctionner sur sur HELTEC ESP32 Wifi/Lora
 *  
 *  Xavier Chavanet
 *  25/03/2020
*/

#include <SSD1306.h>
#include "OLEDDisplayUi.h"
#include "BluetoothSerial.h"

#include <LoRa.h>

#define debug true

#define screenW 128
#define screenH 64

String receivedText;
String receivedRssi;

String entete;
struct dataStructBT {
  char code;
  float vario;
  float nbTours;
  float pression;
};
union BTdataStruct {
    byte b[sizeof(dataStructBT)];
    dataStructBT data;  
};

BTdataStruct BTMessage;

float elevationxs;

char str[132];
long lastTimeValue = millis();
long lastSendlngDevide = millis();


bool jkWathDog = true; 

float fLowpassSlow, fLowpassFast;

const byte led = 25;

// Comptage nombre tour de bobine
const int wheelRotationPin = 35;   // Detection nb tour GPIO35 (flash button for testing)
const int wheelDirectionPin = 34; // Sens de rotation GPIO34 (flash button for testing)
volatile int whellRotationCounter = 0;
volatile int whellDirection = 0;
volatile int whellRotation = 0;

// WIFI_LoRa_32 ports
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)
#define SCK 5
#define CS 18
#define RST 14
#define MISO 19
#define MOSI 27
#define DI0 26
#define BAND    469E6 //433E6 //902E6

// I2C ports
#define SDA 4
#define SCL 15
#define screenResetPin 16

// bips
int freqInit = 550; // hz
int freqGap = 100;
int channel = 0;
int resolution = 8;
int bipVolume = 200; // from 0 to 255
int bBip = 500;
int aBip = -bBip / 6;
bool jkBip = false; 
unsigned long millisBip = millis(); // Gestion du timeOut
boolean BTNeedsToSend;
int nbTrameRec = 0;

// the OLED used
SSD1306 display(0x3c, SDA, SCL);
OLEDDisplayUi ui ( &display );

// Bluetooth
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

// ===========================================================
// Setup
// ===========================================================
void setup() {

  pinMode(led, OUTPUT);
  Serial.begin(115200);
  while (!Serial); //if just the the basic function, must connect to a computer
  delay(100);

  // Initilaisation du mlessage Bluetooth
  BTMessage.data.code='V';
  BTMessage.data.vario=-99;
  BTMessage.data.nbTours=0;
  BTMessage.data.pression=-1;

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
  display.drawString(5, 5, "Vario treuil");
  display.drawString(5, 20, "Version: ");
  display.drawString(5, 30, String(__DATE__) + " " + String(__TIME__) );
 
  display.display();

// ---------------------------------------------------------------------
// Initialisation LoRa
// ---------------------------------------------------------------------
  SPI.begin(SCK, MISO, MOSI, CS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    display.drawString(0, 10, "Starting LoRa failed!");
    while (1);
  }
  
  LoRa.enableCrc();
  LoRa.setSyncWord(0xF2); 
   
// ---------------------------------------------------------------------
// Bluetooth
// ---------------------------------------------------------------------
  SerialBT.begin("VARIO-TREUIL");
  BTNeedsToSend = false;
   
// ---------------------------------------------------------------------
// Comptage du nombre de tour
// ---------------------------------------------------------------------
  pinMode(wheelRotationPin, INPUT_PULLUP); 
  //attachInterrupt(digitalPinToInterrupt(wheelRotationPin), wheelRotationInterrupt, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(wheelRotationPin), wheelInterruptRotation, FALLING); 

  pinMode(wheelDirectionPin, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(wheelDirectionPin), wheelInterruptDirection, FALLING); 
  //nbTours.data.type = 'L';
  //nbTours.data.value = 0 ;

// ---------------------------------------------------------------------
// Initialisation buzzer
// ---------------------------------------------------------------------
  ledcSetup(channel, freqInit, resolution);
  ledcAttachPin(led, channel);
  
  // set buzzer volume
  ledcWrite(channel, bipVolume);

   // Test bip
  for (int i=0;i<5;i++ ){
      tone (channel, 1000);
      delay (150);
      noTone (channel);      
      delay (50);
  }
 
}

//==================================================================
// Boucle principale
//==================================================================
void loop() {

  // try to parse packet
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    // received a packet
     if (debug) {Serial.print("Received packet =>");}

    // read & decode packet
    while (LoRa.available()) {
        String receivedText = LoRa.readString();
        if (debug) {Serial.println(receivedText);}
        
        // Décodage trame. format "vario;tx_1Seconde;tx_xSecondes;pression"
        entete = getValue(receivedText, ';', 0);
        if (entete == "vario") {
            // Decode message and format bluetooth message
            String e1s = getValue(receivedText, ';', 1);
            BTMessage.data.vario=e1s.toFloat();
            
            String exs = getValue(receivedText, ';', 2);

            String pression = getValue(receivedText, ';', 3);
            BTMessage.data.pression=pression.toFloat();
            
            // Display data on lcd
            displayData(BTMessage.data.vario,exs.toFloat());

            BTNeedsToSend = true;
            lastTimeValue = millis();
 
          } else   {      
            displayError (receivedText);
          }
      }    
  }
  
  // Si rien recu depuis 5 secondes, reset vario
  if (lastTimeValue + 5000 < millis()) {
    lastTimeValue = millis() + 100000;
     BTMessage.data.vario=-99; 
     BTMessage.data.pression=-1; 
  }

  // Toutes les secondes, envoi du message BT 
  if (BTNeedsToSend || (lastSendlngDevide + 1000 < millis()) ) {
      BTMessage.data.nbTours = whellRotationCounter;
      if (debug) { 
          Serial.print(" hasClient()=");
          Serial.print(SerialBT.hasClient());
          Serial.print(" Sending BTMessage lng=");
          Serial.println(sizeof(BTMessage.b));
      }
      if (SerialBT.hasClient()) {
        SerialBT.write(BTMessage.b,sizeof(BTMessage.b));
      }
      BTNeedsToSend = false;
      lastSendlngDevide = millis();
}

  // make bip
  makeBips(BTMessage.data.vario); 

  delay (20);
 
}

//========================================================
// getValue (split like)
//========================================================
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//========================================================
// Display data
//========================================================
void displayData (float tx1s, float txxs) {
  int maxT = 4;  
  int x;
  int y;
  
  // Clear screen
  display.clear();
  display.setFont(ArialMT_Plain_10);

  // Watch Dog
  if (jkWathDog) {
    jkWathDog = false;  
    display.drawCircle(5, 5, 2);
  } else
  {
    jkWathDog = true;
  }

  // Echelle
  display.drawLine( 0 , 55, 127 , 55 );
  for( int i=0; i <= maxT; i++ ){
    x = i * ( 128 / maxT ),
    display.drawLine( x  , 50 , x, 63 );
    display.drawString(x -8 , 53, String(i));
  }
    
  // Taux de montée 1 seconde
  if (tx1s < 4 ) {
    display.fillRect( 0 , 10, tx1s * (128/4) , 44 );
  }
  else {
    display.fillRect( (tx1s - 4) * (128/4), 10, 128, 44 );
  }
  
  // Affichage des valeurs
  display.drawString(10 , 0, "tx=");
  display.drawString(25 , 0, String(tx1s));

  receivedRssi = LoRa.packetRssi();
  char currentrs[64];
  receivedRssi.toCharArray(currentrs, 64);
  display.drawString(90, 0, "rssi:" );
  display.drawString(110, 0, currentrs);

  // Affichage
  display.display();
 }   
    
 //========================================================
 // make bips
 //========================================================
void makeBips(float tx) {
  if (tx > 0.5 ) {
      if ( millisBip < millis() ) {
          int timeOut = (aBip * tx) + bBip;
          int freq = freqInit + (freqGap * tx); 
          millisBip = millis() + timeOut;
          if (jkBip) {
            jkBip = false;
            tone (channel, freq);
          } 
          else {
            jkBip = true;
            noTone (channel);            
          } 
      }
  } else {
    noTone (channel);
  }
}
 //========================================================
 // tone (pin freq)
 //========================================================
void tone(uint8_t channel, unsigned int frequency) {
      ledcWriteTone(channel, frequency);
}
 //========================================================
 // notone (pin freq)
 //========================================================
void noTone(uint8_t channel) {
      ledcWriteTone(channel, 0);
}
 //========================================================
 // display erreur
 //========================================================
void displayError (String receivedText) {

   display.clear();
   
   display.setFont(ArialMT_Plain_10);
   display.drawString(0, 2, "Erreur reception trame:" );

   char* data = new char[64];
   receivedText.toCharArray(data, 32 );
   display.drawString(0, 20, data);
   display.display();
}
 //========================================================
 // Detection tour de bobine
 //========================================================
void wheelInterruptRotation () {
  static int oldMillis;
  int t;
  t = millis();
  
  // Minimum de temp de x ms par tour pour eviter les rebonds de values
  if ( (oldMillis + 25) < t ) {
    oldMillis = t;
    if (whellRotation == 1 ) {
        whellRotation = 0;
    } else {
        whellRotation = 1;    
    }

    if (whellRotation == 1){
      if (whellDirection == 1){
        whellRotationCounter ++;
      } else {
        whellRotationCounter --;      
      }

    Serial.print(t);      
    Serial.print(",");
    Serial.print(whellRotation);      
    Serial.print(",");
    Serial.print(whellDirection);      
    Serial.print(",");
    Serial.println(whellRotationCounter);
    }
  }

}
 //========================================================
 // Detection sens de rotation
 //========================================================
void wheelInterruptDirection () {
  static int oldMillis;
  int t;
  t = millis();
  
  // Minimum de temp de x ms par tour pour eviter les rebonds de values
  if ( (oldMillis + 25) < t ) {
    oldMillis = t;
    if (whellDirection == 1 ) {
        whellDirection = 0;
    } else {
        whellDirection = 1;
    }

    Serial.print("direction=>");
    Serial.println(whellDirection);
   }
}
