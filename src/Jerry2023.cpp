#include <Arduino.h>
#include <MPU6050_light.h> //gyro könyvtár
#include <SPI.h>
#include <MFRC522.h>
#include <string.h>
#include <stdio.h>
#include "Wire.h"
#include <PID_v1.h>

//a forráskódban irányok megjelölésére gyakran használatban van a 0 1 2 számozás
#define DIRECTION_FRONT       0 //0-egyenesen
#define DIRECTION_LEFT        1  //1-balra
#define DIRECTION_RIGHT       2 //2-jobbra
#define DIRECTION_STOP        3 //megállás
#define DIRECTION_START       4 //START
#define DIRECTION_DEAD_END    5 //Zsákutca

 
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
double lastDistances[3];
bool isFirstMeasurement = true;
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
int turnMaxSpeed = 120;
int turnMinSpeed = 70;
int turnProportionalSpeed = turnMaxSpeed-turnMinSpeed;

int forwardMaxSpeed = 120;
int forwardMinSpeed = 70;
int forwardProportionalSpeed = forwardMaxSpeed-forwardMinSpeed;


// PID változók   //100 hoz egsz okes:  //60hoz: 
int pidmode = 2;
double setpoint = 0; // Kívánt érték
double input, output;
double Kp1 = 30, Ki1 = 0, Kd1 = 30; // PID tényezők
double Kp2 = 1, Ki2 = 0.1, Kd2 = 1; // PID tényezők
PID pid(&input, &output, &setpoint, Kp2, Ki2, Kd2, DIRECT);



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
  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if(duration > 25000UL) duration = 0;
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
  if(pidmode==1) pid.SetTunings(Kp1,Ki1,Kd1);
  if(pidmode==2) pid.SetTunings(Kp2,Ki2,Kd2);
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


double measureFrontDistanceWithFilter(){
  unsigned int numberOfMeasurements = 4;
  double treshold = 10;
  unsigned int NumberOfMatchesNeeded = 3;
  double frontdistances[numberOfMeasurements];
  for (size_t i = 0; i < numberOfMeasurements; i++)
  {
    frontdistances[i] = measureDistance(TRIGGER_PIN_FRONT, ECHO_PIN_FRONT);
    delay(10);
  }
  for (size_t i = 0; i < numberOfMeasurements; i++)
  {
    int matched = 0;
    for (size_t j = 0; j < numberOfMeasurements; j++)
    {
      if(abs(frontdistances[i]-frontdistances[j])<treshold) matched++;
    }
    if(matched-1>NumberOfMatchesNeeded) return frontdistances[i];
  }
  double sum, avg;
  sum = 0;
  avg = 0;

  for(int i = 0; i < numberOfMeasurements; i++){
      sum += frontdistances[i];
  }
  avg = sum / numberOfMeasurements;
  return avg;
}
//feltölt egy double tömböt távolságokkal - előre, balra és jobbra mér
void measureDistanceAllDirections(){

  distances[DIRECTION_FRONT] = measureFrontDistanceWithFilter();
  delayMicroseconds(500);
  /*if(lastDistances[DIRECTION_FRONT]-distances[DIRECTION_FRONT] > 10 && !isFirstMeasurement){
    distances[DIRECTION_FRONT] = lastDistances[DIRECTION_FRONT];
  }else{
    lastDistances[DIRECTION_FRONT] = distances[DIRECTION_FRONT];
    isFirstMeasurement = false;
  }*/
  distances[DIRECTION_LEFT] = measureDistance(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT);
  delayMicroseconds(500);
  distances[DIRECTION_RIGHT] = measureDistance(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT);
  delayMicroseconds(500);
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

//RFID kártya direction outputtal. -1 értéket ad vissza, ha rossz az olvasás.
  // dirs pointer opcionális az első fordulóban.
  int rfidToDirection(int *dirs = nullptr){
    int retVal[4] = {0};
    if (dirs == nullptr)
    {
      dirs = retVal;
    }
    
    if (mfrc522.PICC_IsNewCardPresent()) {
      if (mfrc522.PICC_ReadCardSerial()) {
        if ((mfrc522.uid.uidByte[1] == 0xBC))
        {
          switch (mfrc522.uid.uidByte[2] & 0xF0)
          {
          case 0xC0:
            /* START */
            dirs[0] = DIRECTION_START;
            break;
          case 0x50:
            /* STOP */
            dirs[0] = DIRECTION_STOP;
            break;
          case 0xF0:
            /* JOBBRA */
            dirs[0] = DIRECTION_RIGHT;
            break;
          case 0x00:
            /* BALRA */
            dirs[0] = DIRECTION_LEFT;
            break;
          case 0x90:
          case 0xA0:
            /* EJJB */
            dirs[0] = DIRECTION_FRONT;
            dirs[1] = DIRECTION_RIGHT;
            dirs[2] = DIRECTION_RIGHT;
            dirs[3] = DIRECTION_LEFT;
          default:
          /* bro baj van */
          dirs[0] = -1;
          break;
        }
      }
      else if (mfrc522.uid.uidByte[1] == 0xBD)
      {
        switch (mfrc522.uid.uidByte[2] & 0xF0)
        {
        case 0xD0:
        case 0xE0:
        case 0xF0:
          /* ZSÁK UTCA */
          dirs[0] = DIRECTION_DEAD_END;
          break;
        case 0x00:
          /* BJJ */
          dirs[0] = DIRECTION_LEFT;
          dirs[1] = DIRECTION_RIGHT;
          dirs[2] = DIRECTION_RIGHT;
          break;
        case 0x60:
          /* JBB */
          dirs[0] = DIRECTION_RIGHT;
          dirs[1] = DIRECTION_LEFT;
          dirs[2] = DIRECTION_LEFT;
          break;
        default:
          /* BRO BAJ VAN */
          dirs[0] = -1;
          break;
        }
      }
      return dirs[0];
    }
  }
  return -2;
}
  
//main loop. ezt ismétli a robot.
void loop() {
  /* RFID matrica olvasása */
  while (false)
  {
    int dir = rfidToDirection();
    switch (dir)
    {
    case DIRECTION_LEFT:
      Serial.println("BAL");
      break;
    case DIRECTION_RIGHT:
      Serial.println("JOBB");
      break;
    case DIRECTION_FRONT:
      Serial.println("ELŐRE");
      break;
    case DIRECTION_DEAD_END:
      Serial.println("ZSÁKUTCA");
      break;
    case -1:
      Serial.println("HIBA 1");
      break;
    case -2:
      /* Nincs kártya */
      Serial.print(".");
      break;
    default:
      Serial.println("Ez nincs még felvíve ebbe a switch case-be");
      break;
    }
  }

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
      
      if (distances[DIRECTION_FRONT] > 5)
      {
        forwardWithAlignment(100);
      }
      else
      {
        stop();        
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
    int intX = (int)(frontDistanceAtTileCenter*10)-140;
    double distanceToTravel = intX % 285;
    distanceToTravel = distanceToTravel/10;
    if(distanceToTravel < 16) distanceToTravel+=28.5;
    
    
    while(distances[DIRECTION_FRONT] > (frontDistanceAtTileCenter-distanceToTravel)){
      
      howFareAreWeFromDestinacion = (distances[DIRECTION_FRONT] - (frontDistanceAtTileCenter-distanceToTravel)) / distanceToTravel;
      
      if(!thereWasANewCommand){
        newCommand = rfidToDirection();
        if(newCommand > 0){
          commands[currentCommand] = newCommand;
          thereWasANewCommand = true;
        }
        
        
      }
      if(howFareAreWeFromDestinacion<0.5){
        pidmode = 2;
      }else{
        pidmode = 2; // 1
      }
      forwardWithAlignment(constrain((forwardMinSpeed+forwardProportionalSpeed*howFareAreWeFromDestinacion), forwardMinSpeed, forwardMaxSpeed));
      measureDistanceAllDirections();
      Serial.print(distances[DIRECTION_FRONT]);
      Serial.print("\t ");
      Serial.print(distanceToTravel);
      Serial.print("\t ");
      Serial.print(frontDistanceAtTileCenter-distanceToTravel);
      Serial.print("\t ");
      Serial.println(howFareAreWeFromDestinacion);
    }
    Serial.println("kilepett");
    drive(-150,-150);
    delay(100);
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
