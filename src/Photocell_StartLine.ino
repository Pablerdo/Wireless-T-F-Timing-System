/* Arduino Photocell System - StartLine module
    by Pablo Salamanca

  Salaman Industries 2019
  
  Terminology
  
  Master = finish line module
*/
#include <RF24Network.h>

#include <Sync.h>
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include "Talkie.h"
#include "Vocab_US_TI99.h"

int CE = 9;
int CSn = 10;

int greenLED = 4;
int blueLED = 5;
int voltLED = 6;
int speaker = 3;

long goSignal = 123; //Number that we will receive when time-taking protocol begins
String signature = "100"; //The electronic signature that prevents interference with other systems.


long masterClock = 0; //Master clock (not synced, there is latency)
long syncedClock = 0; //Synced clock with master
int master_to_S1 = 0; //Time it takes tha message to arrive from master, aprox latencia/2;
long masterActual = 0; //Real time of master
long arrivedTime = 0; //Moment the message arrived from master
long timeSinceDataIn = 0; //Moment data came in last time

Talkie voice;

const uint8_t sp5_REMARK[] PROGMEM = {0x2C, 0xD0, 0xA9, 0xD8, 0xCB, 0xAD, 0xA9, 0x4D, 0x76, 0xE3, 0x74, 0x57, 0x62, 0x56, 0x59, 0x5D, 0x22, 0xCC, 0x89, 0x5B, 0x54, 0x53, 0x2B, 0x73, 0x67, 0x69, 0x8C, 0x15, 0xCB, 0xD2, 0xBD, 0xAD, 0xA6, 0x26, 0x76, 0x31, 0x6B, 0xBB, 0xAA, 0x9E, 0x9C, 0x38, 0xED, 0xCB, 0xCA, 0x66, 0x08, 0x84, 0x8A, 0x35, 0x2D, 0xEB, 0x56, 0x40, 0xAF, 0x16, 0xB7, 0xA4, 0x18, 0xD4, 0x2A, 0x9F, 0xEC, 0x32, 0x6D, 0x74, 0x8A, 0xBC, 0x49, 0x2A, 0x83, 0xE4, 0x1C, 0xF6, 0x38, 0xA5, 0xCA, 0x9A, 0xDA, 0xD9, 0x9D, 0xAC, 0xAA, 0x68, 0xCD, 0x36, 0xB1, 0xB3, 0xAA, 0x10, 0x2E, 0xB5, 0xA8, 0xED, 0xAA, 0xA2, 0xB9, 0xD4, 0xA1, 0x26, 0xAB, 0xCE, 0xFA, 0x42, 0x97, 0xDB, 0xEC, 0x7A, 0xB5, 0x45, 0x9D, 0xE9, 0x51, 0x97, 0x31, 0x14, 0x7B, 0xAA, 0x47, 0x9B, 0xC7, 0x50, 0xFD, 0xB2, 0x2E, 0xF5, 0x46, 0xE0, 0x14, 0xA5, 0x2A, 0x54, 0x03, 0x00, 0xC0, 0x01, 0xBD, 0xB9, 0x2A, 0xA0, 0x79, 0x11, 0x06, 0xB4, 0x62, 0x8C, 0x80, 0x1A, 0x59, 0xFE, 0x1F};
const uint8_t sp3_THREE[] PROGMEM = {0x0C, 0xE8, 0x2E, 0x94, 0x01, 0x4D, 0xBA, 0x4A, 0x40, 0x03, 0x16, 0x68, 0x69, 0x36, 0x1C, 0xE9, 0xBA, 0xB8, 0xE5, 0x39, 0x70, 0x72, 0x84, 0xDB, 0x51, 0xA4, 0xA8, 0x4E, 0xA3, 0xC9, 0x77, 0xB1, 0xCA, 0xD6, 0x52, 0xA8, 0x71, 0xED, 0x2A, 0x7B, 0x4B, 0xA6, 0xE0, 0x37, 0xB7, 0x5A, 0xDD, 0x48, 0x8E, 0x94, 0xF1, 0x64, 0xCE, 0x6D, 0x19, 0x55, 0x91, 0xBC, 0x6E, 0xD7, 0xAD, 0x1E, 0xF5, 0xAA, 0x77, 0x7A, 0xC6, 0x70, 0x22, 0xCD, 0xC7, 0xF9, 0x89, 0xCF, 0xFF, 0x03};
const uint8_t sp3_TWO[]  PROGMEM = {0x06, 0xB8, 0x59, 0x34, 0x00, 0x27, 0xD6, 0x38, 0x60, 0x58, 0xD3, 0x91, 0x55, 0x2D, 0xAA, 0x65, 0x9D, 0x4F, 0xD1, 0xB8, 0x39, 0x17, 0x67, 0xBF, 0xC5, 0xAE, 0x5A, 0x1D, 0xB5, 0x7A, 0x06, 0xF6, 0xA9, 0x7D, 0x9D, 0xD2, 0x6C, 0x55, 0xA5, 0x26, 0x75, 0xC9, 0x9B, 0xDF, 0xFC, 0x6E, 0x0E, 0x63, 0x3A, 0x34, 0x70, 0xAF, 0x3E, 0xFF, 0x1F};

RF24 radio(CE, CSn);

RF24Network network(radio);

const uint16_t master_node  = 00;
const uint16_t S1_node = 01;

int voltPin = A5;

void ledFlash() {
  delay(200);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, HIGH);
  digitalWrite(voltLED, HIGH);
  delay(400);
  digitalWrite(blueLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(voltLED, LOW);

  delay(400);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, HIGH);
  digitalWrite(voltLED, HIGH);

  delay(400);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  digitalWrite(voltLED, LOW);

}


void voltCheck() {
  int rawVoltage = analogRead(voltPin);
  float realVoltage = rawVoltage * (5.00 / 1023.0) * 2;
  if (realVoltage < 7.00) {
    digitalWrite(voltLED, HIGH);
  } else {
    digitalWrite(voltLED, LOW);
  }
}

long decrypt(long encryptedSignalNum) {
  String encryptedSignal = String(encryptedSignalNum);
  if (encryptedSignal.startsWith(signature) == true) {
    encryptedSignal.remove(0, 3);
    long decryptedSignalNum = encryptedSignal.toInt();
    return decryptedSignalNum;
  } else {
    //Means the message wasnt for us
    return -1;
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(voltLED, OUTPUT);
  pinMode(speaker, OUTPUT);


  ledFlash();
  delay(50);
  Serial.begin(9600);
  SPI.begin();

  radio.begin();
  network.begin(124, S1_node);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);

  randomSeed(analogRead(A5));


}

void loop() {
  // put your main code here, to run repeatedly:
  /*Formula to approximately know the real time of the master:
     MasterActual: tiempo en milisegundos desde que maestro partio
     arrivedTime: momento en que nos llego el dato del tiempo sin sincrpnizacion del maestro
     masterClock: tiempo en milisegundos de un momento determinado en maestro (sin sincronizacion)
     master_to_S1: latencia / 2
     syncedClock = masterClock + master_to_S1
     masterActual = (millis() - arrivedTime) + syncedClock

  */

  network.update();

  if (network.available()) {
    RF24NetworkHeader goSignalHeader;
    long encryptedData = 0;
    network.read(goSignalHeader, &encryptedData, sizeof(encryptedData));
    long incomingData = decrypt(encryptedData);
    if ((incomingData != -1) && (goSignalHeader.from_node == 00)) { 
      if (incomingData == goSignal) { //10203
        //Go time

        long startMoment = 0;
        long encryptedStartMoment = 0;
        network.update();
        while (true) {
          network.update();
          if (network.available()) {

            RF24NetworkHeader startMomentHeader;
            network.read(startMomentHeader, &encryptedStartMoment, sizeof(encryptedStartMoment));
            startMoment = decrypt(encryptedStartMoment);
            if (startMoment != -1) {
              break;
            } else {
              encryptedStartMoment = 0;
              startMoment = 0;
              delay(50);
            }
          }
        }
        digitalWrite(blueLED, HIGH);



        long randMarks = random(1000, 3000);



        delay(5);
        tone(speaker, 500, 200);
        delay(300);
        tone(speaker, 500, 200);
        delay(5000);
        digitalWrite(blueLED, LOW);

        voice.say(sp3_THREE);
        delay(randMarks);

        voice.say(sp3_TWO);

        //Pistolazo
        while (((millis() - arrivedTime) + syncedClock) < startMoment) {

        }
        tone(speaker, 320, 500);

      } else if ((incomingData > 2000) && (incomingData != goSignal)) { //If its more than 1500 it means that its not latecy, becasue it would never be as big as 2000 millisecondsSi es mayo
        //Means its the master 
        arrivedTime = millis();
        masterClock = incomingData;

      } else {
        //This means it is latency data

        master_to_S1 = incomingData / 2; //Half because we dont care for the round trip
        syncedClock = masterClock + master_to_S1;

        digitalWrite(greenLED, HIGH);

        masterActual = (millis() - arrivedTime) + syncedClock;
        //Serial.println(masterActual);
        timeSinceDataIn = millis();
        

      }
    }



  }
  if (millis() - timeSinceDataIn > 4000) { //If data doesnt arrive in 4 seconds, something happened to the master and connection is lost
    digitalWrite(greenLED, LOW);
    digitalWrite(blueLED, LOW);
  }
}
