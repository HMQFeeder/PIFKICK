#include <Arduino.h>
#include <vector>
#include <cmath>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <painlessMesh.h> // thư viện để tạo nên mạng Mesh
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// Kích thước màn hình OLED 0.96 inch
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define sda 4
#define scl 5

/*Conversion variables*/
#define EARTH_GRAVITY_MS2 9.80665  //m/s2

// --- CẤU HÌNH THÔNG SỐ ĐO KHOẢNG CÁCH ---
const float ENV_FACTOR = 3.5;         // Hệ số nhiễu môi trường hầm lò (từ 3.0 đến 4.5)
const int SCAN_TIME_SECONDS = 1;      // Mỗi chu kỳ quét sẽ diễn ra trong 1 giây

const char* SSID = "MessWifi"; // khai báo tên mạng mesh
const char* PASSWORD = NULL; // Khai báo mật khẩu của mạng mesh
const uint PORT = 5555; // khai báo cổng thông tin cho mạng mesh, ở đây dùng cổng 5555  
const uint CHANNEL = 6; // khai báo kênh của wifi, dùng kênh 6

const int gas_pin = 15;
const int buzzer = 17;
const int button = 47;

struct oled_element {
    String label;
    int x;
    int y;
    String val;
};

oled_element earth =         {"mpu:"     , 2  , 2 , " "};
oled_element earth_counter = {"wait(s):" , 60 , 2 , " "};
oled_element mq7 =           {"mq7:"     , 2  , 12, " "};
oled_element ble =           {"dist(m):" , 2  , 22, " "};
oled_element fromNode =      {"from:"    , 2  , 37, " "};
oled_element msg =           {"msg:"     , 2  , 47, " "};

// Khởi tạo đối tượng display, dùng chuẩn I2C (&Wire), chân Reset = -1 (không dùng)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Scheduler USVsche; // Khai báo đối tượng để gán task
painlessMesh USVmesh; // Khai báo đối tượng để gán mesh
/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu;
//MPU6050 mpu(0x69); //Use for AD0 high
//MPU6050 mpu(0x68, &Wire1); //Use for AD0 low, but 2nd Wire (TWirI/I2C) object.
BLEScan* pBLEScan; // Khai báo bộ quét BLE

String Smsg = "HELLO WORLD"; 
//khai báo lời nhắn sẽ chuyển (bắt buộc các lời nhắn phải có độ dài bằng nhau để tránh bug trên oled)

uint32_t rootID = 3565053864; // meshID của node root, tìm bằng cách chạy hàm
bool msg_ready = false; // tạo biến báo hiệu xem có tin nhắn sẵn sàng chưa

int8_t MEASURED_POWER;       // RSSI của Beacon khi đứng cách đúng 1 mét
bool is_scanning = false;

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

double aa_z = 0;
double aa_y = 0;
double aa_x = 0;
double aa_total = 0;

uint16_t counter = 0;
double sta_window = 300.0; // tần số lấy mẫu là 100hz => 0.5s
double lta_window = 1500.0; // tương tự => 5s
double sta = 1;
double lta = 1; 
double trig_threshold = 1.5;
double detrig_threshold = 1.0;
double rate = 0;
uint16_t boot_time = 3000; // 30 giây

bool first_data = true;
uint16_t gas_val = 0;
uint16_t gas_threshold = 1000;
bool is_earth_triggered = false;
bool is_gas_triggered = false;
bool is_OLEDalert = false;
bool is_ble_near = false;
bool is_node_triggered = false;

void init_OLED(Adafruit_SSD1306 & display);
void updateOLED(oled_element & element, String new_val, bool is_alert);
void NewConnectionCB(uint32_t newNodeID); // hàm chạy khi node này phát hiện có thêm node khác kết nối vào nó
void RecivedCB(uint32_t fromNodeID, String &Rmsg); // hàm chạy khi node này nhận được tin nhắn từ node khác
void ChangeConnectionCB(); // hàm chạy khi phát hiện sự thay đổi về mặt cấu trúc của mạng (chạy trên toàn bộ node)
void SendMessage(); // hàm gửi thông tin gồm nhu cầu, vị trí của người bị nạn hướng về node gốc
void CheckEarthQuake();
void CheckGas();
void DebugMesh();
void CheckButtonState();
void running();
void buzz();
void buzz_buzz();
float calculateDistance(int rssi);
void scanning();
void FINISHED_SCAN_CB(BLEScanResults foundDevices);

Task BUZZ(TASK_MILLISECOND*100, TASK_FOREVER, &buzz);
Task BUZZ_BUZZ(TASK_MILLISECOND * 300, 4 , &buzz_buzz);

Task SCANNING(TASK_SECOND*4, TASK_FOREVER, &scanning);

Task RUNNING(TASK_MILLISECOND*10, boot_time, &running);
Task CHECK_BUTTON(TASK_MILLISECOND*100, TASK_FOREVER, &CheckButtonState);
Task EARTHQUAKE(TASK_MILLISECOND * 10, 3000 , &CheckEarthQuake);

Task GAS(TASK_MILLISECOND * 500, TASK_FOREVER, &CheckGas);

Task DEBUGGING(TASK_SECOND*5, TASK_FOREVER, &DebugMesh);
Task SEND_MESSAGE(TASK_MILLISECOND*100, TASK_ONCE, &SendMessage); 

// Lớp (Class) xử lý kết quả mỗi khi quét thấy một thiết bị Bluetooth
// Lớp (Class) xử lý kết quả quét - LỌC CHÍNH XÁC THEO UUID MÁY XÚC CỦA BẠN
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      
      // 1. Kiểm tra xem thiết bị quét được có chứa dữ liệu cấu trúc iBeacon không
      if (advertisedDevice.haveManufacturerData()) {
        std::string payload = advertisedDevice.getManufacturerData();
        
        // Gói tin iBeacon chuẩn có độ dài dữ liệu nhà sản xuất là 25 bytes
        if (payload.length() == 25) {
          
          // 2. Tiến hành bóc tách chuỗi UUID từ gói tin (từ byte số 4 đến byte số 19)
          char uuid_str[37];
          snprintf(uuid_str, sizeof(uuid_str), 
                   "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
                   (uint8_t)payload[4],  (uint8_t)payload[5],  (uint8_t)payload[6],  (uint8_t)payload[7],
                   (uint8_t)payload[8],  (uint8_t)payload[9],  (uint8_t)payload[10], (uint8_t)payload[11],
                   (uint8_t)payload[12], (uint8_t)payload[13], (uint8_t)payload[14], (uint8_t)payload[15],
                   (uint8_t)payload[16], (uint8_t)payload[17], (uint8_t)payload[18], (uint8_t)payload[19]);

          String scannedUUID = String(uuid_str);

          // 3. SO SÁNH: Nếu trùng khớp với mã UUID máy xúc trên điện thoại của bạn
          if (scannedUUID.equalsIgnoreCase("E248CCE1-826F-4B1E-ACA4-446B8F19B949")) {
            MEASURED_POWER = (int8_t)payload[24];
            int rssi = advertisedDevice.getRSSI();
            float distance = calculateDistance(rssi);
            String temp = String(calculateDistance(rssi),1);

            Serial.println("\n==================================================");
            Serial.println("Kết nối tới vị trí lân cận");
            Serial.print("   UUID: "); Serial.println(scannedUUID);
            Serial.print("   RSSI: "); Serial.print(rssi); Serial.print(" dBm");
            Serial.print("   Khoảng cách: ~ "); Serial.print(distance); Serial.println(" mét");


            // Logic cảnh báo va chạm an toàn hầm lò
            if (distance > 0 && distance < 2.0) {
              Serial.println(" CẢNH BÁO: Ở quá gần khu vực nguy hiểm (< 2m)!");
              is_ble_near = true;
              updateOLED(ble, temp, true);
              return;
            }
            is_ble_near = false;
            updateOLED(ble, temp, false);
          }
        }
      }
    }
};

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

    USVsche.addTask(BUZZ);
    USVsche.addTask(BUZZ_BUZZ);
    USVsche.addTask(SEND_MESSAGE);
    USVsche.addTask(GAS);
    USVsche.addTask(SCANNING);
    //USVsche.addTask(RUNNING);
    //USVsche.addTask(CHECK_BUTTON);
    //USVsche.addTask(EARTHQUAKE);
    //USVsche.addTask(DEBUGGING);
    
    //CHECK_BUTTON.enable();
    //RUNNING.enable();
    //DEBUGGING.enable();
    BUZZ.enable();
    GAS.enable();
    SCANNING.enable();

    Wire.begin(sda, scl);
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

    // Initializate and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //Supply your gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-4319);
    mpu.setYAccelOffset(-1146);
    mpu.setZAccelOffset(1711);

    //Making sure it worked (returns 0 if so)
    if (devStatus == 0) {
    //mpu.CalibrateAccel(100);  // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateGyro(150);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    MPUIntStatus = mpu.getIntStatus();

    //Set the DMP Ready flag so the main loop() function knows it is okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
    }
    
    Serial.println("--- Hệ thống quét Bluetooth bắt đầu khởi động ---");

    // 2. Khởi tạo thực thể BLE cho chip ESP32
    BLEDevice::init("ACTIVE_SAFETY_SYSTEM");

    // 3. Cấu hình bộ quét BLE Scan
    pBLEScan = BLEDevice::getScan(); 
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); // Gắn hàm xử lý dữ liệu ở trên vào bộ quét
    pBLEScan->setActiveScan(true); // Quét chủ động (Active Scan) để lấy dữ liệu chính xác hơn
    pBLEScan->setInterval(100);    // Tần suất quét (100ms)
    pBLEScan->setWindow(30);       // Thời gian mở cửa sổ quét dữ liệu (90ms)

    init_OLED(display);

    updateOLED(earth, "0.0", false);
    updateOLED(earth_counter, "0", false);
    updateOLED(mq7, "0", false);
    updateOLED(fromNode, "0", false);
    updateOLED(msg, "None", false);
    updateOLED(ble,"0", false);
}

void loop() {
    USVmesh.update();
    if (msg_ready) {
        SEND_MESSAGE.restart();
        msg_ready = false;
    }
}

void init_OLED(Adafruit_SSD1306 & display){
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
    }
    display.clearDisplay();              // Xóa bộ đệm màn hình
    display.setTextSize(1);              // Kích thước chữ (1 là nhỏ nhất, 2 là lớn hơn...)
    display.setTextColor(SSD1306_WHITE); 
}

// Hàm in một chuỗi bất kỳ lên màn hình
void updateOLED(oled_element & element, String new_val, bool is_alert) {
    if (element.val == new_val) return;
    
    display.setCursor(element.x, element.y);    // Đặt con trỏ tại tọa độ x, y
    if (element.val.length() > new_val.length()) {
        uint8_t space = element.val.length() - new_val.length();
        for (int i = 0; i < space ; i++) {
            new_val += ' ';
        }
    }
    element.val = new_val;

    // Đảo màu nền và chữ nếu vượt ngưỡng
    if (is_alert) {
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.print(element.label);
        display.print(new_val);               // Nạp chuỗi vào bộ đệm

    } else {
        display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
        display.print(element.label);
        display.print(new_val);               // Nạp chuỗi vào bộ đệm
 
    }
    display.display();                   // Đẩy bộ đệm ra màn hình thực tế
}

// Hàm toán học tính khoảng cách từ RSSI
float calculateDistance(int rssi) {
  if (rssi == 0) return -1.0;
  // Công thức: Distance = 10 ^ ((Measured_Power - RSSI) / (10 * ))
  return pow(10, (float)(MEASURED_POWER - rssi) / (10 * ENV_FACTOR));
}

void scanning() {
    Serial.println("\n=== Đang quét xung quanh... ===");
    pBLEScan->start(SCAN_TIME_SECONDS, FINISHED_SCAN_CB, false);
}

void FINISHED_SCAN_CB(BLEScanResults foundDevices) {
  pBLEScan->clearResults();
}

void SendMessage() {
    if (USVmesh.sendBroadcast(Smsg)) {
        Serial.println("Gửi thông tin thành công");
    } 
    else {
        Serial.println("Gửi thất bại");
    }
    Smsg = "";
}

void DebugMesh() {
    if (USVmesh.sendBroadcast(F("CheckMic"))) {
        Serial.println("Gửi broadcast thành công");
    }
    else Serial.println("Gửi broadcast failed");
}

void RecivedCB(uint32_t fromNodeID, String &Rmsg) {
    // in ra ID của node gửi tin nhắn tới
    Serial.printf("Nhận được tin nhắn từ %u\n", fromNodeID);
    Serial.print("\n");
    Serial.printf("Lời nhắn là: %s", Rmsg);
    Serial.print("\n");

    String temp = String(fromNodeID);
    if (Rmsg == "EARTHQUAKE!" || Rmsg == "GAS!") {
        is_node_triggered = true;
        updateOLED(fromNode, temp, false);
        updateOLED(msg, Rmsg, true);
    }
    else {
        is_node_triggered = true;
        updateOLED(fromNode, temp, false);
        updateOLED(msg, Rmsg, false);       
    }

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

void CheckEarthQuake() {
    if (counter < 150) {
        counter++;
        return;
    }
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet
        mpu.dmpGetQuaternion(&q, FIFOBuffer);

        mpu.dmpGetAccel(&aa, FIFOBuffer);

        mpu.dmpGetGravity(&gravity, &q);

        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q);

        aa_x = aaWorld.x * (1.0 / 8192.0) * EARTH_GRAVITY_MS2; 
        aa_y = aaWorld.y * (1.0 / 8192.0) * EARTH_GRAVITY_MS2;
        aa_z = aaWorld.z * (1.0 / 8192.0) * EARTH_GRAVITY_MS2;
        aa_total = sqrt(pow(aa_x,2) + pow(aa_y,2) + pow(aa_z,2));

        if (first_data) { // để khi khởi động thì rate không bị lấy 0/0 
            Serial.println("KHOI DONG THANH CONG");
            sta = aa_total;
            lta = aa_total;
            first_data = false;
            return;
        }

        sta = sta + (aa_total - sta)/sta_window;
        if (!is_earth_triggered) { // chặn ở đây để lta không bị nhiễm tín hiệu cao khi có động đất đỡ phải chờ nó lọc 
            lta = lta + (aa_total - lta)/lta_window;
        }
        rate = sta/lta;
        String temp = String(rate, 1);

        if (is_earth_triggered == true) 
        {
            if (rate > detrig_threshold) {
                updateOLED(earth, temp, true);
                Serial.println(rate);
            }
            else {
                is_earth_triggered = false;
                updateOLED(earth, temp, false);
                Serial.println(rate);
            }
        }
        else 
        {
            if (rate > trig_threshold) {
                is_earth_triggered = true;
                updateOLED(earth, temp, true);
                Smsg = "EARTHQUAKE!";
                msg_ready = true;
                Serial.println("===CẢNH BÁO RUNG CHẤN LẠ===");
                return;
            }
            updateOLED(earth, temp, false);
            Serial.println(rate);
        }
    }
}

void CheckGas() {
    gas_val = analogRead(gas_pin);

    if (gas_val > gas_threshold) {
        updateOLED(mq7, (String)gas_val, true);
        if (is_gas_triggered == false) {
            Serial.println("===BÁO ĐỘNG PHÁT HIỆN KHÍ CHÁY===");
            is_gas_triggered = true;
            Smsg = "GAS!";
            msg_ready = true;
        }
    }
    else {
        is_gas_triggered = false;
        Serial.print("giá trị gas là ");
        Serial.println(gas_val);
        updateOLED(mq7, (String)gas_val, false);
    }
}

void CheckButtonState() {
    if (!RUNNING.isLastIteration()) {
        String temp = String(RUNNING.getIterations()/100);
        updateOLED(earth_counter, temp, false);
    }
    else {
        if (digitalRead(button)) {
            first_data = true; 
            counter = 0;
            EARTHQUAKE.restart();
            return;
        }
    }
    if (!RUNNING.isLastIteration()) {
        String temp = String(RUNNING.getIterations()/100);
        updateOLED(earth_counter, temp, false);
    }
}

void running() {
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
        mpu.dmpGetQuaternion(&q, FIFOBuffer);

        mpu.dmpGetAccel(&aa, FIFOBuffer);

        mpu.dmpGetGravity(&gravity, &q);

        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        mpu.dmpConvertToWorldFrame(&aaWorld, &aaReal, &q);
    } 
} 

void buzz() {
    if (is_earth_triggered || is_gas_triggered || is_ble_near) {
        digitalWrite(buzzer, HIGH);
        return;
    }
    else if (is_node_triggered) {
        BUZZ_BUZZ.restart();
        is_node_triggered = false;
    }
    
    if (!BUZZ_BUZZ.isEnabled()) {
        digitalWrite(buzzer, LOW);
    }
}

void buzz_buzz() {
    if (digitalRead(buzzer) == LOW) {
        digitalWrite(buzzer, HIGH);
    } else {
        digitalWrite(buzzer, LOW);
    }
}