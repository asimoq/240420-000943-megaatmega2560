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
#define DIRECTION_START 4 //START

 
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
int commands[256];
int currentCommand;
double howFareAreWeFromDestinacion;

//motor pinek
#define ENA 6 //bal
#define IN1 7
#define IN2 10
#define IN3 4 //jobb
#define IN4 3
#define ENB 5

//motor speedek
int turnMaxSpeed = 110;
int turnMinSpeed = 60;
int turnProportionalSpeed = turnMaxSpeed-turnMinSpeed;

int forwardMaxSpeed = 110;
int forwardMinSpeed = 60;
int forwardProportionalSpeed = forwardMaxSpeed-forwardMinSpeed;


// PID változók   //100 hoz egsz okes: 8 0.01 5 //60hoz: 
double setpoint = 0; // Kívánt érték
double input, output;
double Kp = 1.5, Ki = 0.06, Kd = 1.3; // PID tényezők
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

  commands[0] = 0;
  currentCommand = 0;
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
  
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Trigger jel küldése
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Echo jel értelmezése
  unsigned long duration = pulseIn(echoPin, HIGH, 50000UL);
  if(duration > 50000UL) duration = 0;
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
  float startAngle = mpu.getAngleZ();  //gyro mérése és aktuális állapot mentése
  float currentAngle = mpu.getAngleZ(); 

  drive(-90,90); //fordulás megkezdése
  
  while(currentAngle <= startAngle+170){ //várakozás amíg el nem értük a kívánt fokot. lehet több vagy kevesebb a kívánt fok.
    mpu.update(); //gyro frissítés
    currentAngle = mpu.getAngleZ(); 
    howFareAreWeFromDestinacion = ((startAngle+90) - currentAngle)/90;
    drive(-constrain((turnMinSpeed+(turnProportionalSpeed*howFareAreWeFromDestinacion)),turnMinSpeed,turnMaxSpeed),constrain((turnMinSpeed+(turnProportionalSpeed*howFareAreWeFromDestinacion)),turnMinSpeed,turnMaxSpeed));
  }
  drive(80,-80);
  delay(60);
  stop();   //leállítás mert elértük a kívánt fokot
  
  //jelenlegi helyzet elmentése egy globális változóba. Ezt a helyzetet használjuk egyenesen haladáshoz amennyiben nincsenek falak.
  mpu.update();
  lastCorrectAngle = mpu.getAngleZ();
}

// Jobbra fordulás 90 fok. Magyarázatért look up turnLeft()
void turnRight() {
  mpu.update();
  float startAngle = mpu.getAngleZ();  //gyro mérése és aktuális állapot mentése
  float currentAngle = mpu.getAngleZ(); 

  drive(90,-90);
  while(currentAngle >= startAngle-170){ // lehet több vagy kevesebb a kívánt fok
    mpu.update(); //gyro frissítés
    currentAngle = mpu.getAngleZ(); 
    howFareAreWeFromDestinacion = (currentAngle - (startAngle-90))/90;
    drive(constrain((turnMinSpeed+(turnProportionalSpeed*howFareAreWeFromDestinacion)),turnMinSpeed,turnMaxSpeed),-constrain((turnMinSpeed+(turnProportionalSpeed*howFareAreWeFromDestinacion)),turnMinSpeed,turnMaxSpeed));
  }
  
  drive(-80,80);
  delay(60);

  stop();
  mpu.update();
  lastCorrectAngle = mpu.getAngleZ();
}

//Eldönti hogy az adott irányban van e fal 0cm és 15cm között. Irányt és egy távoságok tömböt vár
bool thereIsAWall(int direction, double distances[]){
  double singleDistance;
  singleDistance= distances[direction];
  if (singleDistance >= 1 && singleDistance <= 16) return true; //22 kb jó
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
  delayMicroseconds(200);
  distances[DIRECTION_LEFT] = measureDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);
  delayMicroseconds(200);
  distances[DIRECTION_RIGHT] = measureDistance(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);
}

//összetett függvény ami a körülötte lévő falak számától függően középre rendezi a robotot miközben előrefele halad. 
void forwardWithAlignment(int maxSpeed) {
  double distanceFromSingleWall = 11.5; //hány cm-re van a fal ha csak egyhez igazodik
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
      } else if (cardData.substring (4,8) == "bd d") {
        Serial.print("palya eleje");
        return DIRECTION_START;
      }
    }
  }
  return 0;
}
  
//main loop. ezt ismétli a robot.
void loop() {
  while (false)
  {
    int asd = rfidToDirection();
  }
  
  
  while (false)
  {
      
      measureDistanceAllDirections();
      mpu.update();
      Serial.print(distances[DIRECTION_LEFT]),
      Serial.print("\t");
      Serial.print(distances[DIRECTION_FRONT]),
      Serial.print("\t");
      Serial.println(distances[DIRECTION_RIGHT]);
      while (false)
      {
        if (distances[DIRECTION_FRONT] > 15)
        {
          forwardWithAlignment(90);
        }
        else
        {
          stop();
          

          delay(500);
          turnLeft();
          turnLeft();
        }
      }
      
  }
  
 

  while (true)
  {
    
    measureDistanceAllDirections();
    double frontDistanceAtTileCenter = distances[DIRECTION_FRONT];
    int newCommand = 0;
    bool thereWasANewCommand = false;
    if(commands[currentCommand] == DIRECTION_LEFT){
      turnLeft();
      delay(1000);
      measureDistanceAllDirections();
      frontDistanceAtTileCenter = distances[DIRECTION_FRONT];
    }
    if(commands[currentCommand] == DIRECTION_RIGHT){
      turnRight();
      delay(1000);
      measureDistanceAllDirections();
      frontDistanceAtTileCenter = distances[DIRECTION_FRONT];
    }
    currentCommand++;
    int intX = (int)(frontDistanceAtTileCenter*10)-96;
    double distanceToTravel = intX % 285;
    distanceToTravel = distanceToTravel/10;
    if(distanceToTravel < 20) distanceToTravel+=28.5;
    
    
    while(distances[DIRECTION_FRONT] > frontDistanceAtTileCenter-distanceToTravel){
      Serial.print(distances[DIRECTION_FRONT]);
      Serial.print("\t ");
      Serial.print(distanceToTravel);
      Serial.print("\t ");
      Serial.println(frontDistanceAtTileCenter-distanceToTravel);
      howFareAreWeFromDestinacion = (distances[DIRECTION_FRONT] - (frontDistanceAtTileCenter-distanceToTravel)) / distanceToTravel;
      
      if(!thereWasANewCommand){
        newCommand = rfidToDirection();
        if(newCommand != 0){
          commands[currentCommand] = newCommand;
          thereWasANewCommand = true;
        }
        thereWasANewCommand = true;
        
      }
      forwardWithAlignment(constrain((forwardMinSpeed+forwardProportionalSpeed*howFareAreWeFromDestinacion), forwardMinSpeed, forwardMaxSpeed));
      measureDistanceAllDirections();
    }
    Serial.println();
    drive(-40,-40);
    delay(50);
    stop();

    if(commands[currentCommand] == 0){
      if(distances[DIRECTION_FRONT] < 28.5){
        if(distances[DIRECTION_LEFT] > distances[DIRECTION_RIGHT]){
          commands[currentCommand] = DIRECTION_LEFT;
        }
        else{
          commands[currentCommand] = DIRECTION_RIGHT;
        }
      }
      else{
        commands[currentCommand] = DIRECTION_FRONT;
      }
    }
    delay(1000);
  }
}
