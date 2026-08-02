

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

/*Conversion variables*/
#define EARTH_GRAVITY_MS2 9.80665  //m/s2
#define DEG_TO_RAD        0.017453292519943295769236907684886
#define RAD_TO_DEG        57.295779513082320876798154814105

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---MPU6050 Control/Status Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gg;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorInt16 ggWorld;    // [x, y, z]            World-frame gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector


void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin(6,5);
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }
  
  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(49);
  mpu.setYGyroOffset(-9);
  mpu.setZGyroOffset(-18);
  mpu.setXAccelOffset(-4209);
  mpu.setYAccelOffset(-1151);
  mpu.setZAccelOffset(1703);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    //mpu.CalibrateAccel(100);  // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  }
}

void loop() {
  if (!DMPReady) return;

  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    /*Display quaternion values in easy matrix form: w x y z */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);

    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetAccel(&aa, FIFOBuffer); // lấy dữ liệu gia tốc từ buffer
    mpu.dmpGetGravity(&gravity, &q); // lấy dữ liệu trọng lực từ quaternion
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // trừ dữ liệu trọng lực khỏi dữ liệu gia tốc
    mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q); // xoay trục tọa độ về trục tọa độ của trái đất ?!
    
    Serial.print("aworld\t");
    Serial.print(aaWorld.x * (1.0 / 8192.0) * EARTH_GRAVITY_MS2); // lấy gia tốc theo trục x nhân với 1 hằng số và nhân với trọng lực (tôi k biết tsao)
    Serial.print("\t");
    Serial.print(aaWorld.y * (1.0 / 8192.0) * EARTH_GRAVITY_MS2);
    Serial.print("\t");
    Serial.println(aaWorld.z * (1.0 / 8192.0) * EARTH_GRAVITY_MS2); //ban đầu dùng 1/8192*2 nhưng đổi lại như vậy vì dmp dùng scale 8192

    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetGyro(&gg, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);
    Serial.print("ggWorld\t");
    Serial.print(ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD);
    Serial.print("\t");
    Serial.print(ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD);
    Serial.print("\t");
    Serial.println(ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD);

    /* Display Euler angles in degrees */
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * RAD_TO_DEG);
    Serial.print("\t");
    Serial.print(ypr[1] * RAD_TO_DEG);
    Serial.print("\t");
    Serial.println(ypr[2] * RAD_TO_DEG);
    
    Serial.println();

  }
}