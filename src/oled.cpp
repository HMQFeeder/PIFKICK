#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Kích thước màn hình OLED 0.96 inch
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Khai báo chân I2C cho ESP32-S3 (Sửa lại nếu bạn cắm chân khác)
#define I2C_SDA 4
#define I2C_SCL 5

// Khởi tạo đối tượng display, dùng chuẩn I2C (&Wire), chân Reset = -1 (không dùng)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Hàm tiện ích để in một chuỗi bất kỳ lên màn hình
void printStringToOLED(String text, int x, int y) {
  display.clearDisplay();              // Xóa bộ đệm màn hình
  display.setTextSize(1);              // Kích thước chữ (1 là nhỏ nhất, 2 là lớn hơn...)
  display.setTextColor(SSD1306_WHITE); // Màu chữ (OLED đơn sắc nên là màu Trắng/Sáng)
  display.setCursor(x, y);             // Đặt con trỏ tại tọa độ x, y
  
  display.println(text);               // Nạp chuỗi vào bộ đệm
  display.display();                   // Đẩy bộ đệm ra màn hình thực tế
}

void setup() {
  Serial.begin(115200);

  // 1. Khởi tạo chuẩn I2C với chân SDA và SCL cụ thể
  Wire.begin(I2C_SDA, I2C_SCL);

  // 2. Khởi tạo màn hình OLED tại địa chỉ I2C 0x3C (địa chỉ phổ biến nhất)
  // Nếu màn hình của bạn không sáng, thử đổi 0x3C thành 0x3D
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("LỖI: Không tìm thấy màn hình OLED SSD1306!"));
    for(;;); // Vòng lặp vô tận, dừng chương trình
  }
  
  Serial.println(F("Khởi tạo OLED thành công!"));

  // 3. Gọi hàm để in chuỗi
  String myText = "Chao ban!\nDay la ESP32-S3\nPlatformIO.";
  printStringToOLED(myText, 0, 10);
}

void loop() {
  // Bạn có thể thêm code cập nhật màn hình ở đây
}