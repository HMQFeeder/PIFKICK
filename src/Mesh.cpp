#include <Arduino.h>
#include <vector>
#include <painlessMesh.h>

Scheduler USVsche;
painlessMesh USVmesh;

const char* SSID = "MessWifi";
const char* PASSWORD = NULL;
const uint PORT = 5555;
const uint CHANNEL = 6;
String Smsg = "";
const int buzzer = 45;

bool msg_ready = false;
bool is_earthquake = false;
bool is_normal = false;

void NewConnectionCB(uint32_t newNodeID);
void RecivedCB(uint32_t fromNodeID, String &Rmsg);
void ChangeConnectionCB();
void SendMessage();
void buzz();
void buzz_buzz();
void CheckMSG();
int counter = 1;

Task BUZZ(TASK_MILLISECOND * 100, TASK_FOREVER, &buzz);
Task BUZZ_BUZZ(TASK_MILLISECOND * 300, 4, &buzz_buzz);
Task CHECK(TASK_SECOND * 2, TASK_FOREVER, &CheckMSG);

void setup() {
    Serial.begin(115200);
    
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, LOW);

    USVmesh.setDebugMsgTypes(ERROR | DEBUG);
    USVmesh.init(SSID, PASSWORD, &USVsche, PORT, WIFI_AP_STA, CHANNEL, 0, 6);
    
    USVmesh.onReceive(&RecivedCB);
    USVmesh.onChangedConnections(&ChangeConnectionCB);
    USVmesh.onNewConnection(&NewConnectionCB);
    
    Serial.print("Node ID của ESP này là: ");
    Serial.println(USVmesh.getNodeId());

    USVsche.addTask(CHECK);
    USVsche.addTask(BUZZ);
    USVsche.addTask(BUZZ_BUZZ);
    BUZZ.enable();

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
    counter++;

    if (USVmesh.sendBroadcast(Smsg)) {
        Serial.println("Gửi thông tin thành công");
    } 
    else {
       Serial.println("Gửi thất bại");
    }
    Serial.println(Smsg);
}

void RecivedCB(uint32_t fromNodeID, String &Rmsg) {
    Serial.printf("Nhận được tin nhắn từ %u\n", fromNodeID);
    Serial.print("\n");
    Serial.printf("Lời nhắn là: %s", Rmsg.c_str());
    Serial.print("\n");

    if (Rmsg == "EARTHQUAKE!") {
        is_earthquake = true;
    } else {
        is_normal = true;
    }
}

void ChangeConnectionCB() {
    String ConnectionTopo = USVmesh.subConnectionJson();
    Serial.println("===CẤU TRÚC MẠNG===");
    Serial.println(ConnectionTopo);
}

void NewConnectionCB(uint32_t newNodeID) {
    Serial.printf("Có thiết bị kết nối vào mạng, ID: %u", newNodeID);
    Serial.print("\n");
}

void buzz() {
    if (is_earthquake) {
        BUZZ_BUZZ.setIterations(16);
        BUZZ_BUZZ.restart();
        is_earthquake = false;
    } else if (is_normal) {
        BUZZ_BUZZ.setIterations(4);
        BUZZ_BUZZ.restart();
        is_normal = false;
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