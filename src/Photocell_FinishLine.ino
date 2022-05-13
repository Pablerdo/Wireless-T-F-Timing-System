// RF24Network - Version: Latest



/*


  Arduino Photocell System - Finish Line Module
   by Pablo Salamanca

   2019
   
  Terminology
   Master = Finish Line Module  
   
   S1 = start line module
   
*/

#include <LiquidCrystal.h>
#include <SPI.h>
#include <nRF24L01.h>

#include <RF24.h>
#include <RF24Network.h>
#include <RF24Network_config.h>
#include <Sync.h>
//-----------------LCD y LED------------//
int lcd_key = 0;
int adc_key_in = 0;

int backlight = 10;
const int btnRIGHT = 0;
const int btnUP = 1;
const int btnDOWN = 2;
const int btnLEFT = 3;
const int btnSELECT = 4;
const int btnNONE = 5;

int senolIsOn = 0; //Variable for cursor control 
int senolUpIsOn = 0; //Variable for cursor control when deciding timer in AUTO mode  

long timeSinceActivity = 0;
bool activity = true;

int LDR1 = A1;
int LDR2 = A2;
int lightStatus = 0; //Variable that tells us how much light there is
int laserNotAligned = 0; //Variable for saving the analog value when the laser is not aligned

int greenLED = A3;
int blueLED = A4;
int redLED = A5;
int laserAlignedValue = 0; //Variable que nos indica cuanto arroja las fotoresistencias cuando el laser esta alineado
const int laserReduction = 150; //Constante que nos dice si el laser ya no esta apuntado. Prototipo

long firstLaserContact = 0; //Variable that measures time since laser is aligned

const int RS = 8, EN = 9, D4 = 4, D5 = 5, D6 = 6, D7 = 7;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

long inMinutes = 0; //Variable that deals the timer's minutes
long inSeconds = 30; //Variable that deals the timer's seconds
bool autoModeChosen = false; //Variable that tells us which interface we have to show
bool countdownEnabled = false; //Variable that tells us if we have to show the countdown
bool countdownOver = false;
long timeSinceEnabled = 0; //Variable hat tells us when the countadown was enabled
long timeToStart = 0; //Time that has to pass until the module has to start the stopwatch
long lastCountdownUpdated = 0; //Variable pto refresh the countdown to some reasonable time

//---------------nRF24L01+ & Networking--------------------//
String signature = "100"; // Number that identifies each system. this has to match in the finish line module and star module
bool wasDisconnected = true;
bool connection = true;
bool overall_connection = true; //Boolean to know if every module is connected
long goSignal = 123; //Numero we send when we want to start stopwatch
long partialTimeRequest = 456;
long encryptedGoSignal = 0; //Numero que mandaremos a traves de la radio frecuencia
long current_time = 0;
long latency = 0;
long before_message = 0;

int partialCount = 0; //Variable that counts the amount of partials
int CE = 2;
int CSn = 3;

RF24 radio(CE, CSn); // 2, 3

RF24Network network(radio);


//This is for the future. I want the system to support partial laser gates
const uint16_t master_node = 00; //Finishline module, master
const uint16_t Start_node = 01; //Startline module, speaker
const uint16_t S2_node = 02; //Partial module
const uint16_t S3_node = 03; //Partial module
const uint16_t S4_node = 04; //Partial Module


uint16_t partialArray[3] = {02, 03, 04};
long nowTime = 0; //Variable para guardar el tiempo en milis en cada momento determinado
long last_sent_start = 0;
long last_sent_partial = 0;


//------------Variables para la toma de tiempo----------//


//---------------Customized Characters--------------//
byte senol[8] = {  
  B10000,
  B11000,
  B11100,
  B11110,
  B11100,
  B11000,
  B10000,

};

byte senolUp[8] = {  
  B00001,
  B00011,
  B00111,
  B01111,
  B11111,
  B01111,
  B11111,
};

int read_lcd_buttons() {
  adc_key_in = analogRead(0);

  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  // We make this the 1st option for speed reasons since it will be the most likely result

  if (adc_key_in > 1000) return btnNONE;

  if (adc_key_in < 50) return btnRIGHT;
  if (adc_key_in < 250) return btnUP;
  if (adc_key_in < 450) return btnDOWN;
  if (adc_key_in < 650) return btnLEFT;
  if (adc_key_in < 850) return btnSELECT;

  return btnNONE;
}


bool laserCheck(bool generalLoop) {

  lightStatus = analogRead(LDR2) + analogRead(LDR1);

  if (generalLoop == false) {
    
    
    lcd.clear();
    lcd.print("Place Module");

    while (lcd_key != btnSELECT) {

      lightStatus = analogRead(LDR1) + analogRead(LDR2);
      laserNotAligned = lightStatus;

      lcd_key = read_lcd_buttons();

      delay(10);
    }

    lcd.clear();
    lcd.print("Point Laser");
    delay(3000);
    firstLaserContact = millis();
    while (millis() - firstLaserContact < 5000) {
      lightStatus = analogRead(LDR1) + analogRead(LDR2);
      if (lightStatus > (laserNotAligned + 150)) {
        lcd.clear();
        lcd.print("Aligned");

        analogWrite(blueLED, 255);

      } else {

        firstLaserContact = millis();
        lcd.clear();
        lcd.print("Misaligned");

        analogWrite(blueLED, 0);
      }
      delay(290);
    }

    lcd.clear();

  } else if (generalLoop == true) {

    if (lightStatus < (laserNotAligned + 150)) {

      firstLaserContact = millis();

      while ((millis() - firstLaserContact) < 5000) {

        lightStatus = analogRead(LDR1) + analogRead(LDR2);
        if (lightStatus >= (laserNotAligned + 300)) {

          lcd.clear();
          lcd.print("Aligned");



          analogWrite(blueLED, 255);
        } else {

          firstLaserContact = millis();

          analogWrite(blueLED, 0);
          lcd.clear();
          lcd.print("Misaligned");

        }
        delay(300);
      }

      lcd.clear();
      lcd.write(byte(0));
      lcd.print("Manual");
      lcd.setCursor(1, 1);
      lcd.print("Auto");
      
      senolIsOn = 0;
      timeSinceActivity = millis();
    }
  }
  analogWrite(blueLED, 0);
}

void ledFlash() {
  delay(200);
  digitalWrite(greenLED, 255);
  digitalWrite(blueLED, 255);
  digitalWrite(redLED, HIGH);
  delay(400);
  digitalWrite(greenLED, 0);
  digitalWrite(blueLED, 0);
  digitalWrite(redLED, LOW);
  delay(400);
  digitalWrite(greenLED, 255);
  digitalWrite(blueLED, 255);
  digitalWrite(redLED, HIGH);
  delay(400);
  digitalWrite(greenLED, 100);
  digitalWrite(blueLED, 0);
  digitalWrite(redLED, LOW);
}




long encrypt(long signalNum) {
  String encryptedSignal = signature + String(signalNum);
  long encryptedSignalNum = encryptedSignal.toInt();
  return encryptedSignalNum;
}

long decrypt(long encryptedSignalNum) {
  String encryptedSignal = String(encryptedSignalNum);
  if (encryptedSignal.startsWith(signature) == true) {
    encryptedSignal.remove(0, 3);
    long decryptedSignalNum = encryptedSignal.toInt();
    return decryptedSignalNum;
  } else {
    //Means this message is not for us
    return -1;
  }
}

void autoInterface() {
  lcd.clear();
  lcd.print("Start: ");
  lcd.print(String(inMinutes));
  lcd.print("m ");
  lcd.print(String(inSeconds));
  lcd.print("s ");
}

void showTotalTime(long min, float sec) {
  lcd.clear();
  if (min == 0) {
    lcd.print("Total: ");
    lcd.setCursor(0, 1);
    lcd.print(String(sec));
    lcd.print("s");
  } else {
    lcd.print("Total: ");
    lcd.setCursor(0, 1);
    lcd.print(String(min));
    lcd.print("m ");
    lcd.print(String(sec));
    lcd.print("s");

  }
}
void timeProtocol() {
  
  //Sends a signal to initiate stopwatch
  network.update();
  RF24NetworkHeader goSignalHeader00(Start_node, 't');
  bool ok5 = network.write(goSignalHeader00, &encryptedGoSignal, sizeof(encryptedGoSignal));
  if (ok5 == false) {
    countdownOver = false;
    countdownEnabled = false;
    autoModeChosen = false;
    lcd.clear();
    lcd.print("ERROR 4");
    delay(3000);
    lcd.clear();
    lcd.write(byte(0));
    lcd.print("Manual");
    lcd.setCursor(1, 1);
    lcd.print("Auto");
    senolIsOn = 0;
    timeSinceActivity = millis();
    return;
  }
  bool ok3 = true;
  network.update();
  if (partialCount > 0) {
    for (int i = 0; i < partialCount; i++) {
      network.update();
      RF24NetworkHeader goSignalHeaderPartial(partialArray[i], 't');
      ok3 = network.write(goSignalHeaderPartial, &encryptedGoSignal, sizeof(encryptedGoSignal));
      if (ok3 == false) {
        countdownOver = false;
        countdownEnabled = false;
        autoModeChosen = false;
        lcd.clear();
        lcd.print("ERROR 5");
        delay(3000);
        lcd.clear();
        lcd.write(byte(0));
        lcd.print("Manual");
        lcd.setCursor(1, 1);
        lcd.print("Auto");
        senolIsOn = 0;
        timeSinceActivity = millis();
        return;
      }
      delay(2);
    }
  }

  long timeLaserBroken = 0;
  if (ok3) {

    delay(10);
    //Choosing th moment when S1 will give the start signal
    network.update();
    long partialTimesArray[3];
    RF24NetworkHeader startMomentHeader(Start_node, 't'); //01
    long timee = millis();
    long startMoment = timee + 12000;
    long encryptedStartMoment = encrypt(startMoment);
    bool ok4 = network.write(startMomentHeader, &encryptedStartMoment, sizeof(encryptedStartMoment));
    if (ok4) {
      lcd.clear();
      lcd.print("Starting...");

      delay(2000);
      //Serial.print("Start Moment: ");
      //Serial.println(startMoment);
      while (millis() < startMoment) {

      }
      lcd.clear();
      lcd.print("Measuring...");
      lcd.setCursor(0, 1);
      lcd.print("DO NOT CROSS");
      int lightBefore = analogRead(LDR1) + analogRead(LDR2);

      while (lightBefore - laserReduction < lightStatus) {
        lightStatus = analogRead(LDR1) + analogRead(LDR2);
      }
      timeLaserBroken = millis();
      lcd.clear();
      lcd.print("Analyzing...");
      delay(2000);
      //Asking the partials to send me their times

      bool ok6;
      //This is for the future when the system will suport multiple laser-gates
      if (partialCount > 0) {
        for (int i = 0; i < partialCount; i++) {
          network.update();
          RF24NetworkHeader partialRequest(partialArray[i], 't');

          long encryptedPartialTimeRequest = encrypt(partialTimeRequest); //456
          bool ok6 = network.write(partialRequest, &encryptedPartialTimeRequest, sizeof(encryptedPartialTimeRequest));
          if (ok6 == false) {
            countdownOver = false;
            countdownEnabled = false;
            autoModeChosen = false;
            lcd.clear();
            lcd.print("ERROR 3");
            delay(3000);
            lcd.clear();
            lcd.write(byte(0));
            lcd.print("Manual");
            lcd.setCursor(1, 1);
            lcd.print("Auto");
            senolIsOn = 0;
            timeSinceActivity = millis();
            return;
          }
        }



        int partialTimesReceived = 0;
        while (partialTimesReceived < partialCount) {
          network.update();
          if (network.available()) {
            RF24NetworkHeader timeReceiver;
            long encryptedPartialTime = 0;
            network.read(timeReceiver, &encryptedPartialTime, sizeof(encryptedPartialTime));
            long decryptedPartialTime = decrypt(encryptedPartialTime);
            if (decryptedPartialTime != -1) {
              if (timeReceiver.from_node == 02) {
                partialTimesArray[0] = decryptedPartialTime;
                partialTimesReceived++;
                Serial.print("Received partial time");
              } else if (timeReceiver.from_node == 03) {
                partialTimesArray[1] = decryptedPartialTime;
                partialTimesReceived++;

              } else if (timeReceiver.from_node == 04) {
                partialTimesArray[2] = decryptedPartialTime;
                partialTimesReceived++;

              }

            }
          }
        }


        lcd.clear();
        int interTimeMinutes[4];
        float interTimeSeconds[4];


        for (int i = 0; i <= partialCount; i++) {
          long thisTime = 0;

          if (i == 0) {
            thisTime = partialTimesArray[i] - startMoment;
          } else if (i == partialCount) { //Medimos el ultimo trazo
            thisTime = timeLaserBroken - partialTimesArray[i - 1];
          } else {
            thisTime = partialTimesArray[i] - partialTimesArray[i - 1];
          }

          interTimeMinutes[i] = floor(thisTime / 60000);
          if (interTimeMinutes[i] == 0) {
            interTimeSeconds[i] = float(thisTime) / 1000.00;

          } else {
            interTimeSeconds[i] = thisTime % 60000;
            interTimeSeconds[i] = float(thisTime) / 1000.00;
          }


        }
        long masterTime = timeLaserBroken - startMoment;
        int masterTimeMinutes = floor(masterTime / 60000);
        float masterTimeSeconds = 0.00;

        if (masterTimeMinutes == 0) {
          masterTimeSeconds = float(masterTime) / 1000.00;

        } else {
          masterTimeSeconds = (masterTime % 60000);
          masterTimeSeconds = float(masterTimeSeconds) / 1000.00;
        }
        lcd.clear();
        int resultsSenol = 1; //1 es 01 - 02 y 02 - 03; 2 es 03 - 04 y 04 - 00; 3 es tiempo total
        lcd_key = read_lcd_buttons();
        int numPage = partialCount + 2;
        while (lcd_key != btnSELECT) {
          lcd_key = read_lcd_buttons();
          if (lcd_key == btnUP) {
            resultsSenol--;
            if (resultsSenol < 1) {
              resultsSenol = 1;
            }
            delay(200);
          } else if (lcd_key == btnDOWN) {
            resultsSenol++;
            if (resultsSenol > (numPage)) {
              resultsSenol = numPage;
            }
            delay(200);
          }


          if (resultsSenol == 1) {
            if (interTimeMinutes[0] == 0) {
              lcd.clear();
              lcd.print("P - 2: ");
              lcd.setCursor(0, 1);
              lcd.print(interTimeSeconds[0]);
              lcd.print("s");
            } else {
              lcd.clear();
              lcd.print("P - 2: ");
              lcd.setCursor(0, 1);
              lcd.print(interTimeMinutes[0]);
              lcd.print("m ");
              lcd.print(interTimeSeconds[0]);
              lcd.print("s");
            }



          } else if (resultsSenol == 2) {
            if (interTimeMinutes[1] == 0) {
              lcd.clear();
              lcd.print("2 - 3: ");
              lcd.setCursor(0, 1);
              lcd.print(interTimeSeconds[1]);
              lcd.print("s");
            } else {
              lcd.clear();
              lcd.print("2 - 3: ");
              lcd.setCursor(0, 1);
              lcd.print(interTimeMinutes[1]);
              lcd.print("m ");
              lcd.print(interTimeSeconds[1]);
              lcd.print("s");
            }
          } else if (resultsSenol == 3) {
            if (resultsSenol != numPage) {
              if (interTimeMinutes[2] == 0) {
                lcd.clear();
                lcd.print("3 - 4: ");
                lcd.setCursor(0, 1);
                lcd.print(interTimeSeconds[2]);
                lcd.print("s");
              } else {
                lcd.clear();
                lcd.print("4 - 4: ");
                lcd.setCursor(0, 1);
                lcd.print(interTimeMinutes[2]);
                lcd.print("m ");
                lcd.print(interTimeSeconds[2]);
                lcd.print("s");
              }
            } else {
              showTotalTime(masterTimeMinutes, masterTimeSeconds);
            }
          } else if (resultsSenol == 4) {
            if (resultsSenol != numPage) {
              if (interTimeMinutes[3] == 0) {
                lcd.clear();
                lcd.print("4 - 5: ");
                lcd.setCursor(0, 1);
                lcd.print(String(interTimeSeconds[3]));
                lcd.print("s");
              } else {
                lcd.clear();
                lcd.print("4 - 5: ");
                lcd.setCursor(0, 1);
                lcd.print(String(interTimeMinutes[3]));
                lcd.print("m ");
                lcd.print(String(interTimeSeconds[3]));
                lcd.print("s");
              }
            } else {
              showTotalTime(masterTimeMinutes, masterTimeSeconds);
            }
          } else {
            showTotalTime(masterTimeMinutes, masterTimeSeconds);
          }

          delay(200);
        }
        lcd.clear();
        lcd.write(byte(0));
        lcd.print("Manual");
        lcd.setCursor(1, 1);
        lcd.print("Auto");
        delay(200);
        timeSinceActivity = millis();
        return;

      } else {
        //Only start and finish line
        lcd.clear();
        long S1MasterTime = timeLaserBroken - startMoment;
        float S1MasterTimeSeconds = 0.00;
        int S1MasterTimeMinutes = floor(S1MasterTime / 60000);
        if (S1MasterTimeMinutes == 0) {
          S1MasterTimeSeconds = float(S1MasterTime) / 1000;
          lcd.print(String(S1MasterTimeSeconds));
          lcd.print("s");
        } else {
          S1MasterTimeSeconds = (S1MasterTime % 60000);
          S1MasterTimeSeconds = float(S1MasterTimeSeconds) / 1000.00;
          lcd.print(String(S1MasterTimeMinutes));
          lcd.print("m ");
          lcd.print(String(S1MasterTimeSeconds));
          lcd.print("s");
        }

        lcd_key = read_lcd_buttons();
        while (lcd_key != btnSELECT) {
          lcd_key = read_lcd_buttons();
          countdownOver = false;
          countdownEnabled = false;
          autoModeChosen = false;

        }

        lcd.clear();
        lcd.write(byte(0));
        lcd.print("Manual");
        lcd.setCursor(1, 1);
        lcd.print("Auto");
        delay(200);
        timeSinceActivity = millis();
        return;
      }
    } else {

      countdownOver = false;
      countdownEnabled = false;
      autoModeChosen = false;
      lcd.clear();
      lcd.print("ERROR 2");
      delay(3000);
      lcd.clear();
      lcd.write(byte(0));
      lcd.print("Manual");
      lcd.setCursor(1, 1);
      lcd.print("Auto");
      senolIsOn = 0;
      timeSinceActivity = millis();

    }

  } else {
    //Error connecting to the partials
    countdownOver = false;
    countdownEnabled = false;
    autoModeChosen = false;
    lcd.clear();
    lcd.print("ERROR 1");
    delay(3000);
    lcd.clear();
    lcd.write(byte(0));
    lcd.print("Manual");
    lcd.setCursor(1, 1);
    lcd.print("Auto");
    senolIsOn = 0;
    timeSinceActivity = millis();


  }
}



void setup() {

  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  delay(100);

  Serial.begin(9600);
  SPI.begin();
  radio.begin();
  network.begin(124, master_node); //Canal RF, este nodo

  lcd.createChar(0, senol);
  lcd.createChar(1, senolUp);
  lcd.begin(16, 2);
  ledFlash();

  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);


  randomSeed(analogRead(A0));

  delay(100);
  lcd.print("Welcome");
  delay(2000);
  lcd.clear();
  delay(300);

  laserCheck(false);

  delay(500);


  lcd_key = read_lcd_buttons();

  delay(200);
  lcd.clear();
  lcd.print("Connecting..");
  delay(2000);


  lcd.clear();
  lcd.write(byte(0));
  lcd.print("Manual");
  lcd.setCursor(1, 1);
  lcd.print("Auto");
  timeSinceActivity = millis();

  //Creating code to maintain communication
  encryptedGoSignal = encrypt(goSignal);
}


void sendVerificationPartial() {

  for (int i = 0; i < partialCount; i++) {
    network.update();
    RF24NetworkHeader verificationPartial(partialArray[i], 't');

    last_sent_partial = millis();
    current_time = millis();

    //Sending real time of the master
    before_message = millis();
    long encryptedCurrentTime = encrypt(millis());
    bool ok = network.write(verificationPartial, &encryptedCurrentTime, sizeof(encryptedCurrentTime));
    if (ok) {

      latency = millis() - before_message; //El tiempo actual menos el tiempo antes de que enviaramos
      network.update();
      RF24NetworkHeader latencyHeaderPartial(partialArray[i], 't');
      long encryptedLatency = encrypt(latency);
      bool ok2 = network.write(latencyHeaderPartial, &encryptedLatency, sizeof(encryptedLatency));
      if (ok2) {
        connection = true;
      } else {
        connection = false;

      }
    } else {
      connection = false;
    }
    //Le mandamos la latencia del maestro al S1


  }

}

void sendVerificationStart() {


  network.update();
  RF24NetworkHeader verification(Start_node, 't'); //Enviandole paquete a nodo 01 (El del parlante)

  last_sent_start = millis();
  current_time = millis();

  //Le enviamos el tiempo actual de maestro
  before_message = millis();
  long encryptedCurrentTime = encrypt(millis());
  bool ok = network.write(verification, &encryptedCurrentTime, sizeof(encryptedCurrentTime));
  if (ok) {

    latency = millis() - before_message; //El tiempo actual menos el tiempo antes de que enviaramos
    network.update();
    RF24NetworkHeader latencyHeader(Start_node, 't');
    long encryptedLatency = encrypt(latency);
    bool ok2 = network.write(latencyHeader, &encryptedLatency, sizeof(encryptedLatency));
    if (ok2) {
      connection = true;
    } else {
      connection = false;

    }
  } else {
    connection = false;
  }
  //Sending latency




}
void loop() {

  //-----Maintaining constant communication with startline module----//
  network.update();
  //Checking startline module

  if (millis() - last_sent_start > 3000) {
    sendVerificationStart();

  }
  if (partialCount > 0) {
    if (millis() - last_sent_partial > 4000) {
      sendVerificationPartial();
    }
  }
  //-----------------------------//

  if (connection == true) {
    if (wasDisconnected == true) {
      lcd.clear();
      lcd.write(byte(0));
      lcd.print("Manual");
      lcd.setCursor(1, 1);
      lcd.print("Auto");
      senolIsOn = 0;
      wasDisconnected = false;
    }
    //---_Checking the laser----//
    laserCheck(true);

    //--------Interfaz LCD---------//


    lcd_key = read_lcd_buttons();

    //If we chose "auto" and the time, we enter this interface
    if ((countdownEnabled == true)) {
      timeSinceActivity = millis();
      if (millis() - lastCountdownUpdated > 400) {
        lastCountdownUpdated = millis();
        long timeLeft = timeToStart - (millis() - timeSinceEnabled) ;

        long timeLeftInMinutes = floor(timeLeft / 60000);

        long timeLeftInSeconds = 0;
        if (timeLeftInMinutes > 0) {
          timeLeftInSeconds = (timeLeft % 60000) / 1000;
        } else {
          timeLeftInSeconds = timeLeft / 1000;
        }
        if ((timeLeftInMinutes == 0) && (timeLeftInSeconds <= 1)) {
          countdownOver = true;
        }
        lcd.clear();
        lcd.print(String(timeLeftInMinutes));
        lcd.print("m ");
        lcd.print(String(timeLeftInSeconds));
        lcd.print("s");
        lcd.setCursor(0, 1);
        lcd.write(byte(0));
        lcd.print("Cancel");
        delay(180);
      }

      if (countdownOver == true) {
        timeProtocol();
        autoModeChosen = false;
        countdownEnabled = false;
        countdownOver = false;
        inMinutes = 0;
        inSeconds = 30;
        timeSinceEnabled = 0;
        timeToStart = 0;
        lcd.clear();
        lcd.write(byte(0));
        lcd.print("Manual");
        lcd.setCursor(1, 1);
        lcd.print("Auto");
        delay(50);
      }

      if (lcd_key == btnSELECT) {
        //We cancel the countdown
        autoModeChosen = false;
        countdownEnabled = false;
        countdownOver = false;

        inMinutes = 0;
        inSeconds = 30;
        timeSinceEnabled = 0;
        timeToStart = 0;
        lcd.clear();
        lcd.write(byte(0));
        lcd.print("Manual");
        lcd.setCursor(1, 1);
        lcd.print("Auto");
        delay(200);
        lcd_key = read_lcd_buttons();
      }

    }
    //Deciding the timer
    if ((autoModeChosen == true) && (countdownEnabled == false) && (activity == true)) {
      timeSinceActivity = millis();
      if (lcd_key == btnRIGHT) {
        if (senolUpIsOn == 0) {
          autoInterface();
          lcd.setCursor(11, 1);
          lcd.write(byte(1));
          senolUpIsOn = 1;
          delay(100);
        }
      } else if (lcd_key == btnLEFT) {
        if (senolUpIsOn == 1) {
          autoInterface();
          lcd.setCursor(8, 1);
          lcd.write(byte(1));
          senolUpIsOn = 0;
          delay(100);
        }
      } else if (lcd_key == btnUP) {
        if (senolUpIsOn == 0) {
          inMinutes++;
          if (inMinutes > 9) {
            inMinutes = 9;
          }
          autoInterface();
          lcd.setCursor(8, 1);
          lcd.write(byte(1));
          delay(190);
        } else {
          inSeconds += 10;
          if (inSeconds > 60) {
            inSeconds = 60;
          }
          autoInterface();
          lcd.setCursor(11, 1);
          lcd.write(byte(1));
          delay(190);
        }
      } else if (lcd_key == btnDOWN) {
        if (senolUpIsOn == 0) {
          inMinutes--;
          if (inMinutes < 0) {
            inMinutes = 0;
          }
          autoInterface();
          lcd.setCursor(8, 1);
          lcd.write(byte(1));
          delay(190);
        } else {
          inSeconds -= 10;
          if (inSeconds < 10) {
            inSeconds = 10;
          }
          autoInterface();
          lcd.setCursor(11, 1);
          lcd.write(byte(1));
          delay(190);

        }
      }
      if (lcd_key == btnSELECT) {
        autoModeChosen = false;
        

        if (inMinutes == 0) {
          timeToStart = inSeconds * 1000;
        } else {
          timeToStart = (inMinutes * 60000) + (inSeconds * 1000);
        }

        timeSinceEnabled = millis();
        countdownEnabled = true;
        delay(250);
        senolIsOn = 0;
        senolUpIsOn = 0;
        read_lcd_buttons();
      }
    }


    if ((lcd_key == btnUP) && (senolIsOn == 1) && (autoModeChosen == false) && (countdownEnabled == false) && (activity == true)) {
      timeSinceActivity = millis();
      lcd.clear();
      lcd.write(byte(0));
      lcd.print("Manual");
      lcd.setCursor(1, 1);
      lcd.print("Auto");
      senolIsOn = 0;
    } else if ((lcd_key == btnDOWN) && (senolIsOn == 0) && (autoModeChosen == false) && (countdownEnabled == false) && (activity == true)) {
      timeSinceActivity = millis();
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Manual");
      lcd.setCursor(0, 1);
      lcd.write(byte(0));
      lcd.print("Auto");
      senolIsOn = 1;
    }

    if ((lcd_key == btnSELECT) && (autoModeChosen == false) && (countdownEnabled == false) && (activity == true)) {
      timeSinceActivity = millis();
      //Start time-taking
      if (senolIsOn == 0) {
        delay(250);
        timeProtocol();


      } else if (senolIsOn == 1) {
        
        autoModeChosen = true;
        autoInterface();
        lcd.setCursor(8, 1);
        lcd.write(byte(1));
        delay(250);
      }
    }


    //Inactivity
    if (millis() - timeSinceActivity > 15000) {
      lcd.noDisplay();
      activity = false;
      digitalWrite(backlight, LOW);
      if (lcd_key != btnNONE) {
        activity = true;
        lcd.display();
        timeSinceActivity = millis();
        delay(200);
        digitalWrite(backlight, HIGH);
      }
    }
  } else {
    wasDisconnected = true;
    lcd.clear();
    lcd.print("Connection Fail");
    lcd.setCursor(0, 1);
    lcd.print("Check Modules");
    delay(300);
    timeSinceActivity = millis();
    lcd.clear();
  }
}

