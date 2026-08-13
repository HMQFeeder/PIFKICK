#include <Arduino.h>
#include <vector>

#include <painlessMesh.h> // thư viện để tạo nên mạng Mesh

Scheduler USVsche; // Khai báo đối tượng để gán task
painlessMesh USVmesh; // Khai báo đối tượng để gán mesh

const char* SSID = "USVMessWifi"; // khai báo tên mạng mesh
const char* PASSWORD = NULL; // Khai báo mật khẩu của mạng mesh
const uint PORT = 5555; // khai báo cổng thông tin cho mạng mesh, ở đây dùng cổng 5555  
const uint CHANNEL = 6; // khai báo kênh của wifi, dùng kênh 6
String Smsg = "HELLO WORLD"; //khai báo lời nhắn sẽ chuyển 
uint32_t rootID = 3565053864; // meshID của node root, tìm bằng cách chạy hàm

bool msg_ready = false; // tạo biến báo hiệu xem có tin nhắn sẵn sàng chưa

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
double detrig_threshold = 1.0;
double rate = 0;
double boot_time = 1000; // 10 giây

bool first_data = true;
bool is_earth_triggered = false;

const int gas_pin = 15;
double gas_val = 0;
const int buzzer = 17;
const int button = 42;

bool is_gas_triggered = false;
uint16_t buzz_timer = 0;

void NewConnectionCB(uint32_t newNodeID); // hàm chạy khi node này phát hiện có thêm node khác kết nối vào nó
void RecivedCB(uint32_t fromNodeID, String &Rmsg); // hàm chạy khi node này nhận được tin nhắn từ node khác
void ChangeConnectionCB(); // hàm chạy khi phát hiện sự thay đổi về mặt cấu trúc của mạng (chạy trên toàn bộ node)
void SendMessage(); // hàm gửi thông tin gồm nhu cầu, vị trí của người bị nạn hướng về node gốc
void CheckMessageState();
void CheckEarthQuake();
void CheckGas();
void DebugMesh();
void CheckButtonState();

Task INIT_EARTHQUAKE(TASK_MILLISECOND*100, TASK_FOREVER, &CheckButtonState);

Task DEBUGGING(TASK_SECOND*5, TASK_FOREVER, &DebugMesh);

Task CHECKMSG(TASK_SECOND*1, TASK_FOREVER, &CheckMessageState);

Task guiTinNhan(TASK_MILLISECOND*100, TASK_ONCE, &SendMessage); 

Task EARTHQUAKE(TASK_MILLISECOND * 10, 2000 , &CheckEarthQuake);

Task GAS(TASK_MILLISECOND * 500, TASK_FOREVER, &CheckGas);

void setup() {    
    Serial.begin(115200);
    
    USVmesh.setDebugMsgTypes(ERROR | DEBUG); // lệnh nhận lỗi
    // khởi động mạng mesh đưa vào SSID, PASSWORD, PORT và đối tượng để gán các task nội bộ vào
    USVmesh.init(SSID, PASSWORD, &USVsche, PORT , WIFI_AP_STA, CHANNEL, 0, 6); 
    
    USVmesh.onReceive(&RecivedCB); // đăng ký sự kiện nhận được tin nhắn
    USVmesh.onChangedConnections(&ChangeConnectionCB); // đăng ký sự kiện có sự thay đổi trong cấu trúc mạng
    USVmesh.onNewConnection(&NewConnectionCB); // đăng ký sự kiện có thiết bị kết nối vào mạng
    
    Serial.print("Node ID của ESP này là: ");
    Serial.println(USVmesh.getNodeId());  //in ra ID của node 

    USVsche.addTask(guiTinNhan);
    USVsche.addTask(CHECKMSG);
    USVsche.addTask(EARTHQUAKE);
    USVsche.addTask(GAS);
    USVsche.addTask(DEBUGGING);
    USVsche.addTask(INIT_EARTHQUAKE);

    INIT_EARTHQUAKE.enable();
    DEBUGGING.enable();
    GAS.enable();
    CHECKMSG.enable();

    Wire.begin(4,5);
    Wire.setClock(400000);

    pinMode(gas_pin, INPUT);
    pinMode(button, INPUT_PULLDOWN);
    pinMode(buzzer, OUTPUT);

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
    is_earth_triggered = false;
    is_gas_triggered = false;
    counter = 0;

}

void loop() {
    USVmesh.update();
    // nếu có tin nhắn sẵn sàng gửi chờ 1 khoản nhỏ rồi gửi
}

void SendMessage() {
    // gửi thông tin người cứu nạn về root
    // nếu root không nằm trong vùng phủ sóng thì sẽ tự động định tuyến đến các root có
    if (USVmesh.sendSingle(rootID, Smsg)) {
        Serial.println("Gửi thông tin tới root node thành công");
    } 
    else {
        Serial.println("Gửi thất bại");
        // Bổ sung thêm lưu dữ liệu vào SD? nếu gửi failed
    }
}

void RecivedCB(uint32_t fromNodeID, String &Rmsg) {
    // in ra ID của node gửi tin nhắn tới
    Serial.printf("Nhận được tin nhắn từ %u\n", fromNodeID);
    Serial.print("\n");
    Serial.printf("Lời nhắn là: %s", Rmsg);
    Serial.print("\n");
}

void ChangeConnectionCB() {
    // lệnh lấy cấu trúc của mạng mesh dưới dạng chuỗi JSON
    String ConnectionTopo = USVmesh.subConnectionJson(); 
    Serial.println("===CẤU TRÚC MẠNG===");
    Serial.println(ConnectionTopo);
}

void NewConnectionCB(uint32_t newNodeID) {
    Serial.printf("Có thiết bị kết nối vào mạng, ID: %u", newNodeID); // gửi ID của thiết bị mới kết nối lên Serial
    Serial.print("\n");
}

void CheckMessageState() {
    if (msg_ready) {
        guiTinNhan.enableDelayed();
    }
}

void CheckEarthQuake() {
    if(!digitalRead(button)) return;

    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
        mpu.dmpGetQuaternion(&q, FIFOBuffer);

        mpu.dmpGetAccel(&aa, FIFOBuffer);

        mpu.dmpGetGravity(&gravity, &q);

        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q);

        avg_aa_x = aaWorld.x * (1.0 / 8192.0) * EARTH_GRAVITY_MS2; 
        avg_aa_y = aaWorld.y * (1.0 / 8192.0) * EARTH_GRAVITY_MS2;
        avg_aa_z = aaWorld.z * (1.0 / 8192.0) * EARTH_GRAVITY_MS2;
        avg_aa_total = sqrt(pow(avg_aa_x,2) + pow(avg_aa_y,2) + pow(avg_aa_z,2));
        
        if (!first_data) { // để khi khởi động thì rate không bị lấy 0/0 
            sta_avg = sta_avg + (avg_aa_total - sta_avg)/sta_window;
            //Serial.print(sta_avg );
            //Serial.print("\t");
            if (!is_earth_triggered) { // chặn ở đây để lta không bị nhiễm tín hiệu cao khi có động đất đỡ phải chờ nó lọc 
                lta_avg = lta_avg + (avg_aa_total - lta_avg)/lta_window;
            }
            //Serial.println(lta_avg);
            rate = sta_avg/lta_avg;
            Serial.println(rate);

            if (rate > trig_threshold && is_earth_triggered == false) {
                Serial.println("===DONG DAT===");
                is_earth_triggered = true;
                digitalWrite(buzzer, HIGH);
            }
            else if (rate < detrig_threshold) {
                Serial.println("KET THUC DONG DAT");
                is_earth_triggered = false;
                digitalWrite(buzzer, LOW);
            }
        }
        else {
            Serial.println("KHOI DONG THANH CONG");
            sta_avg = avg_aa_total;
            lta_avg = avg_aa_total;
            first_data = false;
        }
    }
}

void CheckGas() {
    gas_val = analogRead(gas_pin);

    if (gas_val > 25 && is_gas_triggered == false) {
        Serial.println("===BAO DONG PHAT HIEN KHI CHAY===");
        is_gas_triggered = true;
        digitalWrite(buzzer, HIGH);
    }
    else if ( gas_val <= 25 && is_gas_triggered == true) {
        is_gas_triggered = false;
        digitalWrite(buzzer, LOW);
    }
    Serial.print("giá trị là ");
    Serial.println(gas_val);
}

void DebugMesh() {
    if (USVmesh.sendBroadcast(F("CheckMic"))) {
        Serial.println("Gửi broadcast thành công");
    }
    else Serial.println("Gửi broadcast failed");
}

void CheckButtonState() {
    if (digitalRead(button)) {
        EARTHQUAKE.enable();
    }
}