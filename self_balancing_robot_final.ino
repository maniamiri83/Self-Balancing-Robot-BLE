#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

#define BUFFER_SIZE 20
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
  "00000001-5EC4-4083-81CD-A10B8D5CF6EC",
  BLERead | BLEWrite | BLENotify,
  BUFFER_SIZE,
  false
);

//  PID CONTROLLER STRUCT
typedef struct {
  float Kp, Ki, Kd, tau;
  float limMin, limMax, limMinInt, limMaxInt, T;
  float integrator, prevError, differentiator, prevMeasurement, out;
} PIDController;

void PIDController_Init(PIDController *pid) {
  pid->integrator      = 0;
  pid->prevError       = 0;
  pid->differentiator  = 0;
  pid->prevMeasurement = 0;
  pid->out             = 0;
}
