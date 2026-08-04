
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <cmath>
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWirI/I2C) object.

/*Conversion variables*/
#define EARTH_GRAVITY_MS2 9.80665  //m/s2

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---MPU6050 Control/Status Variables---*/
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 aaReal;
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector

double avg_aa_z = 0;
double avg_aa_y = 0;
double avg_aa_x = 0;
double avg_aa_total = 0;

uint32_t counter = 0;
double sta_window = 50.0; // tần số lấy mẫu là 100hz => 0.5s
double lta_window = 500.0; // tương tự => 5s
double sta_avg = 0.001;
double lta_avg = 0.001; 
double trig_threshold = 1.5;
double detrig_threshold = 1.2;
double rate = 0;
double boot_time = 1000; // 10 giây

bool first_data = true;
bool earthquake = false;
void setup() {

  Wire.begin(6,5);
  Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties

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
  mpu.setXAccelOffset(-4209);
  mpu.setYAccelOffset(-1151);
  mpu.setZAccelOffset(1703);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    //mpu.CalibrateAccel(100);  // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateGyro(150);
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
  first_data = true;
  earthquake = false;
  counter = 0;
}

void loop() {
  if (!DMPReady) return;
  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, FIFOBuffer);

    /* Display initial world-frame acceleration, adjusted to remove gravity
    and rotated based on known orientation from Quaternion */
    mpu.dmpGetAccel(&aa, FIFOBuffer);

    mpu.dmpGetGravity(&gravity, &q);

    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q);

    avg_aa_x = aaWorld.x * (1.0 / 8192.0) * EARTH_GRAVITY_MS2; 
    avg_aa_y = aaWorld.y * (1.0 / 8192.0) * EARTH_GRAVITY_MS2;
    avg_aa_z = aaWorld.z * (1.0 / 8192.0) * EARTH_GRAVITY_MS2;
      
    avg_aa_total = sqrt(pow(avg_aa_x,2) + pow(avg_aa_y,2) + pow(avg_aa_z,2));
    if (counter > boot_time) { // chờ 5 giây trước khi lấy dữ liệu đầu tiên, tránh lta bị bẩn
      if (!first_data) { // để khi khởi động thì rate không bị lấy 0/0 
        sta_avg = sta_avg + (avg_aa_total - sta_avg)/sta_window;
        Serial.print(sta_avg );
        Serial.print("\t");
        if (!earthquake) { // chặn ở đây để lta không bị nhiễm tín hiệu cao khi có động đất đỡ phải chờ nó lọc 
          lta_avg = lta_avg + (avg_aa_total - lta_avg)/lta_window;
          
        }
        Serial.println(lta_avg);
        rate = sta_avg/lta_avg;
        Serial.println(rate);

        if (rate > trig_threshold) {
          Serial.println("===DONG DAT===");
          earthquake = true;
        }
        else if (rate < detrig_threshold) {
          Serial.println("KET THUC DONG DAT");
          earthquake = false;
        }

      }
      else {
        sta_avg = avg_aa_total;
        lta_avg = avg_aa_total;
        first_data = false;
      }
    }
    else if (counter == 0) {
      Serial.printf("DANG KHOI DONG, CHO %i GIAY", (int)boot_time/100);
      Serial.println("");
      counter++;
    }
    else counter++;
  }
}