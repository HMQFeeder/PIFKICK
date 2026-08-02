
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <cmath>
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWI/I2C) object.

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

double sta_avg = 0;
double lta_avg = 1; // để tránh chia cho 0

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

uint32_t counter = 0; 
const uint32_t TARGET_SAMPLES = 500; // Đo 500 mẫu (khoảng 5 giây) là đủ

void loop() {
  if (!DMPReady) return;
  
  Serial.printf("Dang do tan so lay mau qua %u samples...\n", TARGET_SAMPLES);
  
  uint32_t start = millis();
  counter = 0; // Đặt lại bộ đếm mỗi lần đo
  
  while (counter < TARGET_SAMPLES) {
    // get FIFO packet chỉ trả về true khi MPU6050 có dữ liệu mới (mặc định 10ms/lần)
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { 
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetAccel(&aa, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q);
      
      counter++;
    }
    // Ghi chú: Có thể thêm yield(); ở đây nếu dùng ESP để tránh lỗi Watchdog
  }
  
  uint32_t stop = millis();
  uint32_t time_taken = stop - start;
  
  if (time_taken > 0) {
    // Ép kiểu sang số thực (float) để tính toán chính xác
    float sample_rate = (counter * 1000.0) / time_taken;
    
    Serial.printf("Time to take %u samples is %u ms\n", counter, time_taken);
    Serial.printf("Sample frequency is %.2f Hz\n\n", sample_rate);
  }
  
  delay(2000); // Dừng 2 giây để bạn kịp đọc màn hình Serial trước khi đo chu kỳ tiếp theo
}