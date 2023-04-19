#include <SoftwareSerial.h>
SoftwareSerial bluetooth(3,4);

void setup(){
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  bluetooth.begin(38400);
}

void loop(){
  if (bluetooth.available())
    Serial.write(bluetooth.read());
       
  if (Serial.available()){
    String S = GetLine();
    bluetooth.print(S); // Si avec cela le bluetooth ne marche pas éliminer le saut de ligne, remplacer par BT.print(S);
    Serial.println("---> " + S);
  }
}

String GetLine(){
  String S = "" ;
  if (Serial.available()){
    char c = Serial.read(); ;
    while (c != '\n'){
      S = S + c ;
      delay(25) ;
      c = Serial.read();
    }
    return( S ) ;
  }
}
