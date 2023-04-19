#include <SoftwareSerial.h>
#include <NewPing.h>
#include <IRremote.hpp>
#include <Servo.h>

/*****************************************************************************
 Mapping du UNO
******************************************************************************
  Pin 0 (RX) :                          Pin 1 (TX) : 
  Pin 2      : In infrarouge            Pin 3 (PMW): RX BlueTooth
  Pin 4      : TX BlueTooth             Pin 5 (PMW): Moteur Drt port 1
  Pin 6 (PMW): Moteur Drt port 2        Pin 7      : Disponible
  Pin 8      : Disponible               Pin 9 (PMW): Servo
  Pin 10(PMW): PowerMoteur Drt          Pin 11(PWM): PowerMoteur Gch
  Pin 12     : Moteur Gch port 1        Pin 13     : Moteur Gch port 2
  Pin 14 A0  : Trigger Ultrasons        Pin 15 A1  : Echo Ultrasons
  Pin 16 A2  : Capteur IR Gch           Pin 17 A3  : Capteur IR Ctr
  Pin 18 A4  : Capteur IR Drt           Pin 19 A5  : 

******************************************************************************
Muti Tasking via FreeRTOS
******************************************************************************
- Task sensSonar => Ecrit dans la zone distance et head_pos
- Task drive => Lit dans la zone direction SANS SEMAPHORE
- Task SerialCom => Ecrit dans la zone direction et mode
- Task IRCom => Ecrit dans la zone direction et mode
- Task autoDrive => lit distance, head_pos, mode et ecrit dans direction

LA FONCTION SUIVI DE LIGNE N'EST PAS IMPLEMENTEE

******************************************************************************
 DÃ©finition des constantes et variables globales
******************************************************************************
 Variables globales de versionning
*/
const int intMajVersion = 2;
const int intMinVersion = 1;

// Connexion broches moteur
const int MotorRight1 = 6; // Sortie OUT PMW 5 pour le port 1 moteur 1 (droite)
const int MotorRight2 = 5; // Sortie OUT PMW 6 pour le port 2 moteur 1
const int MotorPWDR = 10; // Sortie OUT PMW moteur 1 
const int MotorPWDL = 11; // Sortie OUT PMW moteur 2
const int MotorLeft1 = 12; // Sortie OUT PMW 10 pour le port 1 moteur 2 (gauche)
const int MotorLeft2 = 13; // Sortie OUT PMW 11 pour le port 2 moteur 2

// Code tÃ©lÃ©commande IR
const int irReceiverPin = 2; // Infrarouge connectÃ© Ã  la broche OUT 2
const long IRfront = 0xE718FF00;       // Avant / Touche 2
const long IRback = 0xAD52FF00;       // Arrière / Touche 8
const long IRturnright = 0xA55AFF00;  // Tourner à  Droit / Touche 6
const long IRturnleft = 0xF708FF00;    // Tourner à  Gauche / Touche 4
const long IRstop = 0xE31CFF00;       // Stop / Touche 5
const long IRcny70 = 0xF609FF00;      // Mode automatique CNY70 / Touche EQ
const long IRAutorun = 0xBC43FF00;    // Mode automatique Ultrasons / Touch play pause
const long IRturnsmallleft = 0xF30CFF00; // Touche 1
const long IRStopUtrason = 0xF807FF00; // Touche -
const long IRStartUtrason = 0xEA15FF00; // Touche +


// Brochage capteurs CNY70
const int SensorLeft = A2;      //EntrÃ©e capteur gauche (EntrÃ©e 7)
const int SensorMiddle = A3 ;   //EntrÃ©e capteur centre (EntrÃ©e 4)
const int SensorRight = A4;     //EntrÃ©e capteur droit (EntrÃ©e 3)
int SL;    //Etat capteur gauche
int SM;    //Etat capteur centre
int SR;    //Etat capteur droit

// DÃ©finition pour les Ultrasons
const int USInputPin = A1 ; // Broche rÃ©ception signale RX (broche digitale 13)
const int USOutputPin = A0; // Broche Ã©mission signale TX (broche digitale 12)
int directionn = 0; // Avant=8 AprÃ¨s=2 Gauche=4 Droit=6
int SerPin = 9; //Servo sur PIN OUTPUT Digital 9

// Tableau des distances mesurÃ©es, initialisÃ© Ã  65535 

unsigned int dist_sens[5] = { 65535 , 65535, 65535, 65535, 65535};
unsigned int sensStep = 0;

Servo myservo; //  myservo
const int delay_time = 150; // tempo de stabilisation moteur 

// Activation du mode autorun.
boolean IsAutoRun=false;

// Constantes
const int Fgo = 12; // En avant
const int Rgo = 3; // Tournez Ã  droite
const int Lgo = 9; // Tournez Ã  gauche
const int Bgo = 6; // En arriere
const int Stop= 0; // Robot a l'arrÃªt

// Directions
int direction_actuelle=Stop;
int direction_old=Stop;
int head_pos=90;

boolean scan_enabled=true;

SoftwareSerial bluetooth(3,4);

NewPing sonar(USOutputPin,USInputPin,200);

//******************************************************************************
// Configuration
//******************************************************************************
void setup()
{
  Serial.begin(9600);
  bluetooth.begin(38400);
  pinMode(MotorRight1, OUTPUT);  // Broche 5 (PWM)
  pinMode(MotorRight2, OUTPUT);  // Broche 6 (PWM)
  pinMode(MotorLeft1,  OUTPUT);  // Broche 10 (PWM)
  pinMode(MotorLeft2,  OUTPUT);  // Broche 11 (PWM)
  pinMode(MotorPWDR,  OUTPUT);  // Broche 10 (PWM)
  pinMode(MotorPWDL,  OUTPUT);  // Broche 11 (PWM)
  IrReceiver.begin(irReceiverPin, ENABLE_LED_FEEDBACK);
  pinMode(SensorLeft, INPUT); //DÃ©finition capteur Gauche broche 7
  pinMode(SensorMiddle, INPUT);//DÃ©finition capteur Centre broche 4
  pinMode(SensorRight, INPUT); //DÃ©finition capteur Droite broche 3
  digitalWrite(2, HIGH);
  myservo.attach(SerPin); // Definiton servomoteur sur broche 9 (PWM)
  setupUS();
}

void setupUS()
{
  myservo.write(180);
  delay(100);
  for(int i=0;i<=180;i++)
  {
    myservo.write(i);
    delay(20);
  }
  for(int i=180;i>0;i--)
  {
    myservo.write(i);
    delay(20);
  }  
}

//******************************************************************************
// Mesures des distances sur un angle!
//******************************************************************************
int sense_Angle(int angle) // Retourne la "distance"
{
  myservo.write(angle);
  delay(delay_time);
  head_pos=angle;
  int dist=sonar.ping_cm();
  if(dist==0) {
    dist=65535;
  }
  return dist;
}

void runRobot() {
  if(direction_actuelle!=direction_old) {
    // On commence par stopper le robot
    digitalWrite(MotorRight1, LOW);
    digitalWrite(MotorRight2, LOW);
    digitalWrite(MotorLeft1, LOW);
    digitalWrite(MotorLeft2, LOW);
    digitalWrite(MotorPWDR, HIGH);
    digitalWrite(MotorPWDL, HIGH);
    // Et aprÃ¨s on traite
    if(direction_actuelle==Fgo) {
      digitalWrite(MotorRight1, LOW);
      digitalWrite(MotorRight2, HIGH);
      digitalWrite(MotorLeft1, LOW);
      digitalWrite(MotorLeft2, HIGH);
      digitalWrite(MotorPWDR, HIGH);
      digitalWrite(MotorPWDL, HIGH);
      direction_old=Fgo;
    } else if(direction_actuelle==Rgo){
      digitalWrite(MotorRight1, HIGH);
      digitalWrite(MotorRight2, LOW);
      digitalWrite(MotorLeft1, LOW);
      digitalWrite(MotorLeft2, HIGH);
      digitalWrite(MotorPWDR, HIGH);
      digitalWrite(MotorPWDL, HIGH);
      for(int i=4;i<0;i--) {
        dist_sens[i]=(unsigned int)((dist_sens[i]+dist_sens[i-1])/2);
      }
      direction_old=Rgo;
    } else if(direction_actuelle==Lgo) {
      digitalWrite(MotorRight1, LOW);
      digitalWrite(MotorRight2, HIGH);
      digitalWrite(MotorLeft1, HIGH);
      digitalWrite(MotorLeft2, LOW);
      digitalWrite(MotorPWDR, HIGH);
      digitalWrite(MotorPWDL, HIGH);
      for(int i=0;i<4;i++) {
        dist_sens[i]=(unsigned int)((dist_sens[i]+dist_sens[i+1])/2);
      }
      direction_old=Lgo;
    } else if(direction_actuelle==Stop) {
      digitalWrite(MotorRight1, LOW);
      digitalWrite(MotorRight2, LOW);
      digitalWrite(MotorLeft1, LOW);
      digitalWrite(MotorLeft2, LOW);
      digitalWrite(MotorPWDR, LOW);
      digitalWrite(MotorPWDL, LOW);
      direction_old=Stop;
    } else if(direction_actuelle==Bgo) {
      digitalWrite(MotorRight1, HIGH);
      digitalWrite(MotorRight2, LOW);
      digitalWrite(MotorLeft1, HIGH);
      digitalWrite(MotorLeft2, LOW);
      digitalWrite(MotorPWDR, HIGH);
      digitalWrite(MotorPWDL, HIGH);
      direction_old=Bgo;
    }
  }
}


boolean handleDirection(char msg) {
  if(msg=='a') {
    IsAutoRun=false;
    direction_actuelle=Fgo;
  }else if (msg=='r') {
    IsAutoRun=false;
    direction_actuelle=Bgo;
  }else if (msg=='d') {
    IsAutoRun=false;
    direction_actuelle=Rgo;
  }else if (msg=='g') {
    IsAutoRun=false;
    direction_actuelle=Lgo;
  }else if (msg=='s') {
    IsAutoRun=false;
    direction_actuelle=Stop;
  }else if (msg=='u') {
    IsAutoRun=true;
    scan_enabled=true;
  /*    }else if (msg=='l') {
    xSemaphoreGive(directionSemaphore); 
    lineFollower();
   */
  }else if (msg=='m') {
    scan_enabled=not scan_enabled;
  }else if (msg=='x') {
    Serial.print("direction :");
    Serial.println(direction_actuelle);
    bluetooth.print("direction :");
    bluetooth.println(direction_actuelle);
  } else {
    return false;
  }
  return true;
}

//******************************************************************************
// Progamme de suivi de ligne noir au sol.
//******************************************************************************
/* A debugger
void lineFollower()
{
  boolean IsCNY70=true;
  while (IsCNY70)
  {
    SL = digitalRead(SensorLeft);
    SM = digitalRead(SensorMiddle);
    SR = digitalRead(SensorRight);

    if (SM == HIGH)//Capteur centrale dans le noir ?
    {
      if (SL == LOW & SR == HIGH) // Gauche blanc, droit noir ? Tourner Ã  gauche...
      {
        digitalWrite(MotorRight1, LOW);
        digitalWrite(MotorRight2, HIGH);
        analogWrite(MotorLeft1, 0);
        analogWrite(MotorLeft2, 80);
      }
      else if (SR == LOW & SL == HIGH) //Droit blanc, gauche noir ? Tourner Ã  droit :)
      {
        analogWrite(MotorRight1, 0);
        analogWrite(MotorRight2, 80);
        digitalWrite(MotorLeft1, LOW);
        digitalWrite(MotorLeft2, HIGH);
      }
      else  // Droit et gauche dans la mÃªme couleur ? en avant.
      {
        advance(0);
        analogWrite(MotorLeft1, 200);
        analogWrite(MotorLeft2, 200);
        analogWrite(MotorRight1, 200);
        analogWrite(MotorRight2, 200);
      }
    }
    else // Si le centrale est dans du blanc et :
    {
      if (SL == LOW & SR == HIGH)// Gauche blanc, droit noir, virage rapide Ã  gauche
      {
        left(0);
      }
      else if (SR == LOW & SL == HIGH) // Droit blanc, gauche noir, virage rapide Ã  droite.
      {
        right(0);
      }
      else // Tout est blanc ? STOP !
      {
        back(0);
      }
    }
    if (irrecv.decode(&results))
    {
      irrecv.resume();
      IsCNY70 = false;
      if (results.value == IRstop)
      {
        stopp(0);
        break;
      }
    }
    String dataStream;
    if(Serial.available()) {
      dataStream = Serial.readStringUntil('\n');
      if(dataStream.equals("ST")) {
        Serial.println("OK");
        stopp(0);
        IsCNY70 = false;
        break;
      }
    }
  }
}
*/

//******************************************************************************(LOOP)
void loop()
{
  if(Serial.available())
  {      
    int received=Serial.read();
    if(handleDirection(received)) {
      Serial.println("OK");
      bluetooth.println("OK");
    } else {
      Serial.println("ER");
      bluetooth.println("ER");
    }
  }
  if(bluetooth.available())
  {      
    int received=bluetooth.read();
    if(handleDirection(received)) {
      Serial.println("OK");
      bluetooth.println("OK");
    } else {
      Serial.println("ER");
      bluetooth.println("ER");
    }
  }
  if (IrReceiver.decode())
  { 
    char msg=' ';
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
    if (IrReceiver.decodedIRData.decodedRawData == IRfront)//Avance ?
    {
      msg='a';  
    }
    if (IrReceiver.decodedIRData.decodedRawData ==  IRback)//Marche arriÃ¨re ?
    {
      msg='r';  
    }
    if (IrReceiver.decodedIRData.decodedRawData == IRturnright)// Droite
    {
      msg='d';  
    }
    if (IrReceiver.decodedIRData.decodedRawData == IRturnleft)//Gauche ?
    {
      msg='g';  
    }
    if (IrReceiver.decodedIRData.decodedRawData == IRstop)//On coupe tout !
    {
      msg='s';  
    }
/*    if (results.value == IRcny70)
    {
      msg='l';  
    } */
    if (IrReceiver.decodedIRData.decodedRawData == IRAutorun  )
    {
      msg='u';  
    }
    if (IrReceiver.decodedIRData.decodedRawData == IRStartUtrason )
    {
      msg='m';
    }
    if(msg!=' ') {
      if(handleDirection(msg)) {
        Serial.println("OK");
        bluetooth.println("OK");
      } else {
        Serial.println("ER");
        bluetooth.println("ER");
      }
    }
    IrReceiver.resume();
  }
  if(scan_enabled) {
    sensSonar();
  }
  autoDrive();
  runRobot();
}

void sensSonar() {
  unsigned int sensAngle[8] = { 0 , 45, 90, 135, 180, 135 , 90, 45 };
  unsigned int realStep;

  if(sensStep<5) {
    realStep=sensStep;
  } else {
    realStep=8-sensStep;
  } 
  dist_sens[realStep]=(unsigned int)((dist_sens[realStep]+(unsigned int)sense_Angle(sensAngle[sensStep])*2)/3);
/*
  Serial.print(sensStep);
  Serial.print("-");
  Serial.print(sensAngle[sensStep]);
  Serial.print("-");
  Serial.println(dist_sens[sensStep]);

  bluetooth.print(sensStep);
  bluetooth.print("-");
  bluetooth.print(sensAngle[sensStep]);
  bluetooth.print("-");
  bluetooth.println(dist_sens[sensStep]);
*/
  
  sensStep++;
  if(sensStep>=8) {
    sensStep=0;
  }
}


void autoDrive() {
  int sens_distance;
  if (IsAutoRun)
  {
    scan_enabled=true;
    if(direction_actuelle==Lgo) {
      sens_distance = min(min(dist_sens[4],dist_sens[2])*2,dist_sens[3]);
    } else if (direction_actuelle==Rgo) {
      sens_distance = min(min(dist_sens[0],dist_sens[2])*2,dist_sens[1]);
    } else {
      sens_distance = min(min(dist_sens[1],dist_sens[3])*2,dist_sens[2]);
    }

    if (sens_distance <= 10) // Si la distance est infÃ©rieur Ã  10cm en face.
    {
       direction_actuelle=Bgo; 
    } else if (sens_distance <= 25) // Si la distance est infÃ©rieur Ã  25cm en face.
    {

      if (min(dist_sens[4],dist_sens[3]) > min(dist_sens[0],dist_sens[1])) //Si la distance Gauche est plus grande que la droite
      {
        direction_actuelle=Lgo; //On va Ã  gauche !
      }
  
      if (min(dist_sens[4],dist_sens[3]) <= min(dist_sens[0],dist_sens[1])) //Si la distance droite est plus grande ou Ã©gale Ã  gauche
      {
        direction_actuelle=Rgo; // On va Ã  droite !
      }
  
      if (min(dist_sens[4],dist_sens[3]) < 15 && min(dist_sens[0],dist_sens[1]) < 15) // Si les distance droite et gauche sont infÃ©rieurs a 10cm
      {
        direction_actuelle=Bgo; //En arriere
      }
    } else //Si non, si on a plus de 25cm en face
    {
      direction_actuelle=Fgo; //En avant.
    }
  }    
}
