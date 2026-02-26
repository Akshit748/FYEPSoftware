#include <Arduino.h>

void sendObstacleAlert(){
  Serial.println("Obstacle Alert Sent");
}

void sendEmergency(){
  Serial.println("Emergency SMS Sent!");}

//for now this only sends messages in the serial monitor, this file and its contents serve as a placeholder for future features
