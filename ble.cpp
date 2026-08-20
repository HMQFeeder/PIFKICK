#include<Arduino.h>

#include<math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// --- CẤU HÌNH THÔNG SỐ ĐO KHOẢNG CÁCH ---
const float ENV_FACTOR = 3.5;         // Hệ số nhiễu môi trường hầm lò (từ 3.0 đến 4.5)
const int SCAN_TIME_SECONDS = 2;      // Mỗi chu kỳ quét sẽ diễn ra trong 2 giây
int8_t MEASURED_POWER;
bool is_scanning = false;
BLEScan* pBLEScan; // Khai báo bộ quét BLE

// Hàm toán học tính khoảng cách từ RSSI
float calculateDistance(int rssi) {
  if (rssi == 0) return -1.0;
  // Công thức: Distance = 10 ^ ((Measured_Power - RSSI) / (10 * n))
  return pow(10, (float)(MEASURED_POWER - rssi) / (10 * ENV_FACTOR));
}
void FINISHED_SCAN_CB(BLEScanResults foundDevices) {
  // 5. Xóa kết quả quét cũ trong bộ nhớ RAM sau mỗi chu kỳ để tránh tràn bộ nhớ
  pBLEScan->clearResults();
  is_scanning = false;
}
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
            
            int rssi = advertisedDevice.getRSSI();
            MEASURED_POWER = (int8_t)payload[24];
            float distance = calculateDistance(rssi);

            // --- IN THÔNG TIN ĐÍCH DANH MÁY XÚC LÊN MONITOR ---
            Serial.println("\n==================================================");
            Serial.println("ĐÃ PHÁT HIỆN MÁY XÚC CỦA BẠN!");
            Serial.print("   UUID: "); Serial.println(scannedUUID);
            Serial.print("   RSSI: "); Serial.print(rssi); Serial.print(" dBm");
            Serial.print("   Khoảng cách: ~ "); Serial.print(distance); Serial.println(" mét");

            // Logic cảnh báo va chạm an toàn hầm lò
            if (distance > 0 && distance < 2.0) {
              Serial.println("   ⚠️ CẢNH BÁO NGUY HIỂM: Xe xúc đang ở QUÁ GẦN (< 2m)!");
            }
            Serial.println("==================================================");
          }
        }
      }
    }
};


void setup() {
  // 1. Khởi chạy cổng Serial với tốc độ Baudrate 115200
  Serial.begin(115200);
  delay(1000); // Đợi 1 giây để cổng Serial ổn định
  Serial.println("--- Hệ thống quét Bluetooth bắt đầu khởi động ---");

  // 2. Khởi tạo thực thể BLE cho chip ESP32
  BLEDevice::init("ACTIVE_SAFETY_SYSTEM");

  // 3. Cấu hình bộ quét BLE Scan
  pBLEScan = BLEDevice::getScan(); 
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); // Gắn hàm xử lý dữ liệu ở trên vào bộ quét
  pBLEScan->setActiveScan(true); // Quét chủ động (Active Scan) để lấy dữ liệu chính xác hơn
  pBLEScan->setInterval(100);    // Tần suất quét (100ms)
  pBLEScan->setWindow(90);       // Thời gian mở cửa sổ quét dữ liệu (90ms)
}

void loop() {
  
  // 4. Bắt đầu quét và đợi kết quả trong khoảng thời gian chỉ định
  if (!is_scanning) {
    Serial.println("\n=== Đang quét xung quanh... ===");
    pBLEScan->start(SCAN_TIME_SECONDS, FINISHED_SCAN_CB, false);
    is_scanning = true;
  }
}