#include <Arduino.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <MFRC522.h>
#include <string.h>
#include <stdio.h>
#include "Wire.h"
#include <PID_v1.h>

//a forráskódban irányok megjelölésére gyakran használatban van a 0 1 2 számozás
//0-egyenesen
//1-balra
//2-jobbra


//gyro
MPU6050 mpu(Wire);
unsigned long timer = 0;
float lastCorrectAngle = 0;

//ultrahangos pinek
#define TRIGGER_PIN_FRONT 2
#define ECHO_PIN_FRONT 3
#define TRIGGER_PIN_RIGHT 4
#define ECHO_PIN_RIGHT 5
#define TRIGGER_PIN_LEFT 6
#define ECHO_PIN_LEFT 7

//motor pinek
//bal
#define ENA 11
#define IN1 29
#define IN2 27
//jobb
#define IN3 25
#define IN4 23
#define ENB 10

// PID változók
double setpoint = 0; // Kívánt érték
double input, output;
double Kp = 1, Ki = 0.1, Kd = 0.1; // PID tényezők



PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

#define RST_PIN 8
#define SS_PIN 9

MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

void setup() {
  Serial.begin(9600);
  //gyro beállítása
  Wire.begin();
   byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // Ultrahangos távérzékelő pin-ek beállítása
  pinMode(TRIGGER_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(TRIGGER_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  pinMode(TRIGGER_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);

  // Motorvezérlő pin-ek beállítása
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // RFID kártyaolvasó inicializálása
  SPI.begin();
  mfrc522.PCD_Init();
  // Prepare key - all keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
}

// motorbeállítás
void drive(int motorSpeedLeft, int motorSpeedRight) {
  // Motor A irányának beállítása
  if (motorSpeedLeft >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    motorSpeedLeft = -motorSpeedLeft;
  }

  // Motor B irányának beállítása
  if (motorSpeedRight >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    motorSpeedRight = -motorSpeedRight;
  }

  // Motorok sebességének beállítása
  analogWrite(ENA, motorSpeedLeft);
  analogWrite(ENB, motorSpeedRight);
}


//TÁVOLSÁGMÉRÉS CM-BEN
double measureDistance(int triggerPin, int echoPin) {
  // Előkészítés
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Trigger jel küldése
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Echo jel értelmezése
  long duration = pulseIn(echoPin, HIGH);
  // Távolság kiszámítása a hangsebesség alapján
  double distance = duration * 0.034 / 2; // A hangsebesség 34 cm/ms, a távolságot pedig két irányban kell elosztani
  
  if(distance > 200){ //hibás mérést 0-ra állít mert valszeg túl közel értünk a falhoz.
    distance = 0;
  }
  return distance;
}

// Előre haladás
void forward() {
  drive(80,80);
}

// Hátramenet
void backward() {
  drive(-80,-80);
}
// Balra fordulás 90 fok
void turnLeft() {
  mpu.update();
  float angle = mpu.getAngleZ();
  drive(-80,80);
  while(mpu.getAngleZ() <= angle+90){ // lehet több vagy kevesebb a kívánt fok
    mpu.update();
  }
  stop();
  mpu.update();
  lastCorrectAngle = mpu.getAngleZ();
}

// Jobbra fordulás 90 fok
void turnRight() {
  mpu.update();
  float angle = mpu.getAngleZ();
  drive(80,-80);
  while(mpu.getAngleZ() >= angle-90){ // lehet több vagy kevesebb a kívánt fok
    mpu.update();
  }
  mpu.update();
  lastCorrectAngle = mpu.getAngleZ();
}

// Megállás
void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}


bool thereIsAWall(int direction){
  double distance;
  if(direction == 0){
    distance = measureDistance(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT);
  }
  else if(direction == 1){
    distance = measureDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);
  }
  else{
    distance = measureDistance(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);
  }
  if (distance >= -1 && distance <= 15) return true;
  else return false;
}

void PidDrive(double middleDistance){
  input = middleDistance;

  // Számold ki a PID szabályozó kimenetét
  pid.Compute();

  // Motorok vezérlése a PID kimenet alapján
  int motorSpeedLeft = constrain(255 + output, 0, 255); // Bal motor sebessége
  int motorSpeedRight = constrain(255 - output, 0, 255); // Jobb motor sebessége

  // Motorok mozgatása
  drive(motorSpeedLeft,motorSpeedRight);

  //gyro pozíció mentése
  mpu.update();
  lastCorrectAngle = mpu.getAngleZ();

  // Késleltetés a következő ciklusig
  delay(100);
}

void forwardWithAlignment() {
  double distanceFromSingleWall = 15; //hány cm-re van a fal ha csak egyhez igazodik
  //mindkét oldalt van fal
  if(thereIsAWall(1) && thereIsAWall(2)){
    double rightDistance = measureDistance(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);
    double leftDistance = measureDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);

    // Számold ki a középső távolságot a jobb és bal oldali távolságok alapján
    double middleDistance = (rightDistance - leftDistance) / 2.0;

    PidDrive(middleDistance);
  }
  //balra van csak fal
  if(thereIsAWall(1) && !thereIsAWall(2)){
    double leftDistance = measureDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);

    // Számold ki a középső távolságot a jobb és bal oldali távolságok alapján
    double middleDistance = (distanceFromSingleWall - leftDistance) / 2.0;
    
    PidDrive(middleDistance);
  }
  //jobbra van csak fal
  if(!thereIsAWall(1) && thereIsAWall(2)){
    double rightDistance = measureDistance(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);
    

    // Számold ki a középső távolságot a jobb és bal oldali távolságok alapján
    double middleDistance = (rightDistance - distanceFromSingleWall) / 2.0;

    PidDrive(middleDistance);
  }
  //Nincs fal mellette
  else{
    mpu.update();
    float angle = mpu.getAngleZ();
    double middleDistance = (lastCorrectAngle - angle) / 2.0; //gyro alapján egyenesen a legutóbbi helyezkedéstől(flhoz igazítás vagy fordulás) számolva tartja a szöget elvileg :D
    PidDrive(middleDistance);
  }

}

//RFID kártya 0, 1, 2, 3-vége outputtal
int rfidToDirection(){
  if (mfrc522.PICC_IsNewCardPresent()) {
    if (mfrc522.PICC_ReadCardSerial()) {
      // RFID kártya adatok kiolvasása
      String cardData = "";
      for (byte i = 0; i < mfrc522.uid.size; i++) {
        cardData += (String)(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
        cardData += String(mfrc522.uid.uidByte[i], HEX);
      }
      Serial.print("Sima: ");
      Serial.print(cardData);
      Serial.println();
      String vagott = cardData.substring (4,8);
      Serial.print("vagott: ");
      Serial.print(vagott);
      Serial.println();
      

      // Kártya adatok alapján műveletek végrehajtása
      if (cardData.substring (4,8) == "bc 0") {
        Serial.print("fordulas balra");
        return 1;
      } else if (cardData.substring (4,8) == "bc f") {
        Serial.print("fordulas jobbra");
        return 2;
      } else if (cardData.substring (4,8) == "bc 5") {
        Serial.print("palya vege");
        return 3;
      }
    }
  }
  return 0;
}
  



void loop() {
  // put your main code here, to run repeatedly:
  //gyro update
  mpu.update();
  // RFID kártyaolvasó ellenőrzése
  if (mfrc522.PICC_IsNewCardPresent()) {
    if (mfrc522.PICC_ReadCardSerial()) {
      // RFID kártya adatok kiolvasása
      String cardData = "";
      for (byte i = 0; i < mfrc522.uid.size; i++) {
        cardData += (String)(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
        cardData += String(mfrc522.uid.uidByte[i], HEX);
      }
      Serial.print("Sima: ");
      Serial.print(cardData);
      Serial.println();
      String vagott = cardData.substring (4,8);
      Serial.print("vagott: ");
      Serial.print(vagott);
      Serial.println();
      

      // Kártya adatok alapján műveletek végrehajtása
      if (cardData.substring (4,8) == "bc 0") {
        Serial.print("fordulas balra: ");
        forwardWithAlignment();
        
        delay(200);
        stop();
        delay(100);
        turnLeft();
        
      } else if (cardData.substring (4,8) == "bc f") {
        Serial.print("fordulas jobbra: ");
        forwardWithAlignment();
        
        delay(200);
        stop();
        delay(100);
        turnRight();
        
      } else if (cardData.substring (4,8) == "bc 5") {
        // Program kilépése
        stop();
        while (true) {
          // Végtelen ciklusban várakozás
        }
      }
    }
  }

  // Első távérzékelő mérése cm-ben
  float frontDistance = measureDistance(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT);
  //Serial.println(frontDistance);
  
  if (frontDistance <= 5){
    stop();
    
    while(true){}
  }
  // Fal érzékelése az első távérzékelővel
  if (frontDistance <= 15) {
    
    
    // Fal érzékelése, fordulás az irányban, ahol több hely van
    if (measureDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT) > measureDistance(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT)) {
      backward();
      delay(500);
      turnLeft();
      
    } else {
      backward();
      delay(500);
      turnRight();
      
    }
  } else {
    // Nincs fal az előtt, előre megyünk középre igazítással
    forwardWithAlignment();
  }
}
