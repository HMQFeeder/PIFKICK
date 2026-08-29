#include <Arduino.h>
#include <vector>

#include <painlessMesh.h> // thư viện để tạo nên mạng Mesh

Scheduler USVsche; // Khai báo đối tượng để gán task
painlessMesh USVmesh; // Khai báo đối tượng để gán mesh

const char* SSID = "MessWifi"; // khai báo tên mạng mesh
const char* PASSWORD = NULL; // Khai báo mật khẩu của mạng mesh
const uint PORT = 5555; // khai báo cổng thông tin cho mạng mesh, ở đây dùng cổng 5555  
const uint CHANNEL = 6; // khai báo kênh của wifi, dùng kênh 6
String Smsg = ""; //khai báo lời nhắn sẽ chuyển 
const int buzzer = 3;

bool msg_ready = false; // tạo biến báo hiệu xem có tin nhắn sẵn sàng chưa
bool is_node_triggered = false;
// hàm khởi tạo tin nhắn cứu hộ

void NewConnectionCB(uint32_t newNodeID); // hàm chạy khi node này phát hiện có thêm node khác kết nối vào nó
void RecivedCB(uint32_t fromNodeID, String &Rmsg); // hàm chạy khi node này nhận được tin nhắn từ node khác
void ChangeConnectionCB(); // hàm chạy khi phát hiện sự thay đổi về mặt cấu trúc của mạng (chạy trên toàn bộ node)
void SendMessage(); // hàm gửi thông tin gồm nhu cầu, vị trí của người bị nạn hướng về node gốc
void buzz();
void buzz_buzz();
void CheckMSG();
int counter = 1;

Task BUZZ(TASK_MILLISECOND*100, TASK_FOREVER, &buzz);
Task BUZZ_BUZZ(TASK_MILLISECOND * 150, 4 , &buzz_buzz);
Task CHECK(TASK_SECOND*2, TASK_FOREVER, &CheckMSG);

// tạo task gửi tin nhắn
// chờ 100 milisecon trc khi thực thi task
// thực thi 1 lần rồi tắt
// gọi hàm SendMessage khi thực thi 
//Task guiTinNhan(TASK_MILLISECOND*100, TASK_ONCE, &SendMessage); 

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

    //USVsche.addTask(guiTinNhan);
    USVsche.addTask(CHECK);
    USVsche.addTask(BUZZ);
    USVsche.addTask(BUZZ_BUZZ);

    CHECK.enable();
}

void loop() {
    USVmesh.update();
}

void CheckMSG() {
    if (counter == 1) {
        Smsg = "mic check";
    }
    else if (counter == 2) {
        Smsg = "EARTHQUAKE!";
    }
    else if (counter == 3) {
        Smsg = "GAS!";
        counter = 0;
    }
    counter ++;

    if (USVmesh.sendBroadcast(Smsg)) {
        Serial.println("Gửi thông tin thành công");
    } 
    else {
       Serial.println("Gửi thất bại");
    }
    Serial.println(Smsg);

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

void buzz() {
if (is_node_triggered) {
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