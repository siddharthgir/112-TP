import processing.net.*; 
import java.util.Arrays;
import processing.serial.*;
import cc.arduino.*;
/* This file receives data from the inverse kinematics program 
and relays those values to the arduino*/


Arduino arduino;

int rightChest = 2;  // attaches the servo on pin 9 to the servo object
int rightShoulder = 7;
int rightArm = 8;
int rightElbow = 11;
int leftChest = 13;
int leftShoulder = 12;
int leftArm = 10;
int leftElbow = 9;
//int hip = 4;

//Client side code learned from: https://processing.org/reference/libraries/net/Client.html

Client myClient; 
int incoming[] = new int[]{0,0,0,0,0,0,0,0};

void setup() { 
  size(200, 200); 
  println(Arduino.list());
  arduino = new Arduino(this, "com11", 57600);
  arduino.pinMode(rightChest, Arduino.SERVO);
  arduino.pinMode(rightShoulder, Arduino.SERVO);
  arduino.pinMode(rightArm, Arduino.SERVO);
  arduino.pinMode(rightElbow, Arduino.SERVO);
  arduino.pinMode(leftChest, Arduino.SERVO);
  arduino.pinMode(leftShoulder, Arduino.SERVO);
  arduino.pinMode(leftArm, Arduino.SERVO);
  arduino.pinMode(leftElbow, Arduino.SERVO);
  //arduino.pinMode(hip, Arduino.SERVO);

  myClient = new Client(this, "127.0.0.1", 10000); 
} 
 
void draw() { 
  while (myClient.available() >= 8) { 
    for(int i =0; i < 8; i++){
    incoming[i] = myClient.read();
    }
    arduino.servoWrite(rightChest, incoming[0]);
    arduino.servoWrite(rightShoulder, incoming[1]);
    arduino.servoWrite(rightArm, incoming[2]);
    arduino.servoWrite(rightElbow, incoming[3]);
    arduino.servoWrite(leftChest, incoming[4]);
    arduino.servoWrite(leftShoulder, incoming[5]);
    arduino.servoWrite(leftArm, incoming[6]);
    arduino.servoWrite(leftElbow, incoming[7]);
    //arduino.servoWrite(hip, 95);
    delay(5);
  } 
  
}
