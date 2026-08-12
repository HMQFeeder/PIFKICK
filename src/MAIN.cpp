#include <Arduino.h>
#include <vector>

#include <painlessMesh.h> // thư viện để tạo nên mạng Mesh
#include <ArduinoJson.h> // thư viện để tạo tin nhắn cứu hộ

Scheduler USVsche; // Khai báo đối tượng để gán task
painlessMesh USVmesh; // Khai báo đối tượng để gán mesh
JsonDocument Sdoc; // khai báo đối tượng Json để gửi đi
JsonDocument Rdoc; // khai báo đối tượng Json nhận vào

const char* SSID = "USVMessWifi"; // khai báo tên mạng mesh
const char* PASSWORD = NULL; // Khai báo mật khẩu của mạng mesh
const uint PORT = 5555; // khai báo cổng thông tin cho mạng mesh, ở đây dùng cổng 5555  
const uint CHANNEL = 6; // khai báo kênh của wifi, dùng kênh 6
String Smsg = ""; //khai báo lời nhắn sẽ chuyển 
uint32_t rootID = 3565053864; // meshID của node root, tìm bằng cách chạy hàm

bool msg_ready = false; // tạo biến báo hiệu xem có tin nhắn sẵn sàng chưa

// hàm khởi tạo tin nhắn cứu hộ
void init_Sdoc(JsonDocument & Sdoc) {
    Sdoc["ho_ten"] = ""; // tạo key ho_ten, value là trống
    Sdoc["dac_diem_noi_tru_an"] = ""; // tạo key dac_diem_noi_tru_an, value là trống
    Sdoc["nhu_cau"].to<JsonArray>(); // tạo key nhu_cau là một mảng, chưa chứa gì hết
    Sdoc["FromNodeID"] = ""; // tạo key ID của node gửi, val là 
}

void NewConnectionCB(uint32_t newNodeID); // hàm chạy khi node này phát hiện có thêm node khác kết nối vào nó
void RecivedCB(uint32_t fromNodeID, String &Rmsg); // hàm chạy khi node này nhận được tin nhắn từ node khác
void ChangeConnectionCB(); // hàm chạy khi phát hiện sự thay đổi về mặt cấu trúc của mạng (chạy trên toàn bộ node)
void SendMessage(); // hàm gửi thông tin gồm nhu cầu, vị trí của người bị nạn hướng về node gốc
//void PrintConnectionTree(); // hàm in ra cấu trúc của mạng mesh hiện tại


void CheckMSG() {
    // chuyển tin nhắn từ kiểu dữ liệu JSON (Sdoc) sang kiểu dữ liệu String (Smsg) 
    // để gửi tới root node
    Smsg = "mic check";
    if (USVmesh.sendSingle(rootID, Smsg)) {
        Serial.println("Gửi thông tin tới root node thành công");
    } 
    else {
       Serial.println("Gửi thất bại");
        // Bổ sung thêm lưu dữ liệu vào SD? nếu gửi failed
    }
}

Task CHECK(TASK_SECOND*3, TASK_FOREVER, &CheckMSG);

// tạo task gửi tin nhắn
// chờ 100 milisecon trc khi thực thi task
// thực thi 1 lần rồi tắt
// gọi hàm SendMessage khi thực thi 
Task guiTinNhan(TASK_MILLISECOND*100, TASK_ONCE, &SendMessage); 

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
    USVsche.addTask(CHECK);
    CHECK.enable();
}

void loop() {
    USVmesh.update();
    // nếu có tin nhắn sẵn sàng gửi chờ 1 khoản nhỏ rồi gửi
    if (msg_ready) {
        guiTinNhan.enableDelayed();
        msg_ready = false;
    }
}

void SendMessage() {
    // chuyển tin nhắn từ kiểu dữ liệu JSON (Sdoc) sang kiểu dữ liệu String (Smsg) 
    // để gửi tới root node
    serializeJson(Sdoc, Smsg);
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
    /*
    // chuyển từ kiểu dữ liệu String sang kiểu dữ liệu JSON
    // nhập JSON vào Rdoc
    deserializeJson(Rdoc, Rmsg); 
    Serial.println(F("====CÓ NGƯỜI CẦN CỨU HỘ===="));
    // đọc họ tên người gửi
    String ho_ten = Rdoc["ho_ten"];
    Serial.printf("Họ và tên: %s\n", ho_ten);
    // đọc đặc điểm nơi trú ẩn của người gửi
    String dac_diem_noi_tru_an = Rdoc["dac_diem_noi_tru_an"];
    Serial.printf("Đặc điểm nơi trú ẩn: %s\n", dac_diem_noi_tru_an);
    // Nhập toàn bộ nhu cầu của người gửi sang một mảng
    JsonArray array = Rdoc["nhu_cau"];
    // đọc số lượng nhu cầu
    int so_luong_nhu_cau = array.size();

    if ( so_luong_nhu_cau == 0) {
        Serial.println(F("User không gửi nhu cầu nào"));
        return;
    }
    // khởi tạo một vector để nhận nhu cầu của người dùng
    std::vector<String> nhu_cau;
    // nhập toàn bộ nhu cầu của người dùng vào một vector
    for (JsonVariant v : array) {
        nhu_cau.push_back(v);
    }
    Serial.print(F("Nhu cầu: "));
    for (String item : nhu_cau) {
        Serial.print(item);
        Serial.print(", ");
    }
    Serial.print("\n");
    // xử lý xong rồi giờ muốn làm gì tiếp thì làm
    */
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

// làm test đo độ trễ từ lúc rút dây 1 node cho đến khi event change connection xuát hiện