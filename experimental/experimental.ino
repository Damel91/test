
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

double totalSpeed;
double acc;
double pitch;
float adjustAngle = 0.00;
float speedRobot = 0.00;
unsigned long dmpTimer;

MPU6050 mpu;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 aa; // [x, y, z]            accel sensor measurements
VectorInt16 aaReal; // [x, y, z]            gravity-free accel sensor measurements



void setup() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  // initialize device
  mpu.initialize();

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    mpuIntStatus = mpu.getIntStatus();

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  dmpData();
  Serial.println(pitch);
  Serial.println(totalSpeed);
}



void dmpData() {
  //unsigned long now = millis();
  //if (now - dmpTimer >= sampleTime) {
  //dmpTimer = now;
  //sensorDataSync = true;
  mpuIntStatus = mpu.getIntStatus();
  if (mpuIntStatus & 0x02 || fifoCount >= packetSize) {
    // reset interrupt flag and get INT_STATUS byte
    //dmpReady = false;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      // Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
      float ypr[3];
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      ypr[1] = ypr[1] * 180 / M_PI;
      pitch = adjustAngle + ypr[1] + speedRobot;
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      //speed = a * t
      unsigned long now = millis();
      if (now - dmpTimer > 5) {
        dmpTimer = now;
        double time = dmpTimer / 1000;
        acc = aaReal.x /** cos(fabs(pitch))*/;
        //precAcc = acc;
        double speed = time * acc;
        /*
          if (acc < 0.00) {
          if (acc > precAcc) {
            totalSpeed = totalSpeed + fabs(speed);
          } else {
            totalSpeed = speed + totalSpeed;
          }
          } else {
          if (acc < precAcc) {
            totalSpeed = totalSpeed - speed;
          } else {
            totalSpeed = speed + totalSpeed;
          }
          }
        */
        totalSpeed = speed + totalSpeed;
      }
    }
  }
}
