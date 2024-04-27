#include <Arduino.h>
#include <MPU6050_light.h> //gyro könyvtár
#include <SPI.h>
#include <MFRC522.h>
#include <string.h>
#include <stdio.h>
#include "Wire.h"
#include <PID_v1.h>

//a forráskódban irányok megjelölésére gyakran használatban van a 0 1 2 számozás
#define DIRECTION_FRONT 0 //0-egyenesen
#define DIRECTION_LEFT 1  //1-balra
#define DIRECTION_RIGHT 2 //2-jobbra
#define DIRECTION_STOP 3 //megállás
 
//gyro
MPU6050 mpu(Wire);
unsigned long timer = 0;
float lastCorrectAngle = 0;

//ultrahangos pinek
#define TRIGGER_PIN_FRONT A2
#define ECHO_PIN_FRONT A3
#define TRIGGER_PIN_RIGHT A0
#define ECHO_PIN_RIGHT A1
#define TRIGGER_PIN_LEFT A4
#define ECHO_PIN_LEFT A5

double distances[3];

//motor pinek
#define ENA 6 //bal
#define IN1 7
#define IN2 8
#define IN3 3 //jobb
#define IN4 4
#define ENB 5

// PID változók
double setpoint = 0; // Kívánt érték
double input, output;
double Kp = 8, Ki = 0.01, Kd = 5; // PID tényezők
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//RFID CONFIG
#define RST_PIN 8
#define SS_PIN 9
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

//robot inicializálása
void setup() {
  Serial.begin(115200);
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

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255,255);

  mpu.update();
  lastCorrectAngle = mpu.getAngleZ();
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
  /* double measurements[3];
  for (size_t i = 0; i < 3; i++)
  {
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
    measurements[i] = duration * 0.034 / 2;
    
  }
  double distance;
  double min = abs(measurements[0]-measurements[1]);
  distance = (measurements[0]+measurements[1])/2;
  double second = abs(measurements[1]-measurements[2]);
  double third = abs(measurements[2]-measurements[0]);
  if(min > second)
  {
    min = second;
    distance = (measurements[1]+measurements[2])/2;
  }
  if(min > third)
  {
    distance = (measurements[2]+measurements[0])/2;
  }
  return distance; */

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
  if(triggerPin == TRIGGER_PIN_RIGHT) Serial.println(duration);
  if(duration > 20000) duration = 0;
  if(duration == 0) return 0;
  // Távolság kiszámítása a hangsebesség alapján
  return duration * 0.034 / 2;
  

  
}

// Előre haladás
void forward() {
  drive(80,80);
}

// Hátramenet
void backward() {
  drive(-80,-80);
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

// Balra fordulás 90 fok
void turnLeft() {

  mpu.update();                   //gyro frissítése
  float angle = mpu.getAngleZ();  //gyro mérése és aktuális állapot mentése

  drive(-80,80); //fordulás megkezdése
  
  while(mpu.getAngleZ() <= angle+90){ //várakozás amíg el nem értük a kívánt fokot. lehet több vagy kevesebb a kívánt fok.
    mpu.update(); //gyro frissítés
  }
  stop();   //leállítás mert elértük a kívánt fokot
  
  //jelenlegi helyzet elmentése egy globális változóba. Ezt a helyzetet használjuk egyenesen haladáshoz amennyiben nincsenek falak.
  mpu.update();
  lastCorrectAngle = mpu.getAngleZ();
}

// Jobbra fordulás 90 fok. Magyarázatért look up turnLeft()
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

//Eldönti hogy az adott irányban van e fal 0cm és 15cm között. Irányt és egy távoságok tömböt vár
bool thereIsAWall(int direction, double distances[]){
  double singleDistance;
  singleDistance= distances[direction];
  if (singleDistance >= 1 && singleDistance <= 22) return true; //22 kb jó
  else return false;
}

//PID alapján beállítja a motorok sebességét
void PidDrive(double distanceFromMiddle, int maxSpeed, bool isThereAWall){
  input = distanceFromMiddle;

  // Számold ki a PID szabályozó kimenetét
  pid.Compute();

  // Motorok vezérlése a PID kimenet alapján
  int motorSpeedLeft = constrain(maxSpeed - output, -255, 255); // Bal motor sebessége
  int motorSpeedRight = constrain(maxSpeed + output, -255, 255); // Jobb motor sebessége

  // Motorok mozgatása
  drive(motorSpeedLeft,motorSpeedRight);

  //jelenlegi helyzet elmentése egy globális változóba. Ezt a helyzetet használjuk egyenesen haladáshoz amennyiben nincsenek falak.
  if(isThereAWall){
    lastCorrectAngle = mpu.getAngleZ();
  }
}

//feltölt egy double tömböt távolságokkal - előre, balra és jobbra mér
void measureDistanceAllDirections(){
  distances[DIRECTION_FRONT] = measureDistance(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT);
  distances[DIRECTION_LEFT] = measureDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);
  distances[DIRECTION_RIGHT] = measureDistance(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);
}

//összetett függvény ami a körülötte lévő falak számától függően középre rendezi a robotot miközben előrefele halad. 
void forwardWithAlignment(int maxSpeed) {
  double distanceFromSingleWall = 11; //hány cm-re van a fal ha csak egyhez igazodik
  //mindkét oldalt van fal
  if(thereIsAWall(DIRECTION_LEFT, distances) && thereIsAWall(DIRECTION_RIGHT, distances)){

    // Számold ki a középső távolságot a jobb és bal oldali távolságok alapján
    double distanceFromMiddle = (distances[DIRECTION_RIGHT] - distances[DIRECTION_LEFT]) / 2.0;

    PidDrive(distanceFromMiddle, maxSpeed, true);
  }
  //balra van csak fal
  if(thereIsAWall(DIRECTION_LEFT, distances) && !thereIsAWall(DIRECTION_RIGHT, distances)){

    // Számold ki a középső távolságot a jobb és bal oldali távolságok alapján
    double distanceFromMiddle = (distanceFromSingleWall - distances[DIRECTION_LEFT]) / 2.0;
    
    PidDrive(distanceFromMiddle, maxSpeed, true);
  }
  //jobbra van csak fal
  if(!thereIsAWall(DIRECTION_LEFT, distances) && thereIsAWall(DIRECTION_RIGHT, distances)){

    // Számold ki a középső távolságot a jobb és bal oldali távolságok alapján
    double distanceFromMiddle = (distances[DIRECTION_RIGHT] - distanceFromSingleWall) / 2.0;

    PidDrive(distanceFromMiddle, maxSpeed, true);
  }
  //Nincs fal mellette 
  if(!thereIsAWall(DIRECTION_LEFT, distances) && !thereIsAWall(DIRECTION_RIGHT, distances)){
    float angle = mpu.getAngleZ();
    double error = (angle -lastCorrectAngle) * 0.07; //gyro alapján egyenesen a legutóbbi helyezkedéstől(falhoz igazítás vagy fordulás) számolva tartja a szöget elvileg :D
    PidDrive(error, maxSpeed, false);
  }

}

//RFID kártya direction outputtal
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
        return DIRECTION_LEFT;
      } else if (cardData.substring (4,8) == "bc f") {
        Serial.print("fordulas jobbra");
        return DIRECTION_RIGHT;
      } else if (cardData.substring (4,8) == "bc 5") {
        Serial.print("palya vege");
        return DIRECTION_STOP;
      }
    }
  }
  return 0;
}
  
//main loop. ezt ismétli a robot.
void loop() {
  measureDistanceAllDirections();
  mpu.update();
  Serial.print(distances[DIRECTION_LEFT]),
  Serial.print(" ");
  Serial.print(distances[DIRECTION_FRONT]),
  Serial.print(" ");
  Serial.println(distances[DIRECTION_RIGHT]);
  if (distances[DIRECTION_FRONT] > 15)
  {
    forwardWithAlignment(100); //100 kb jó
    /* code */
  }
  else
  {
    drive(0,0);
  }
  
  
  
  /* // put your main code here, to run repeatedly:
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
  } */
}
