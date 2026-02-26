#ifndef SENSOR_H
#define SENSOR_H

struct SensorData(){
  float irDistance;
  float ultrasonicDistance;
};

void initSensors():
SensorData readSensors();

#endif
