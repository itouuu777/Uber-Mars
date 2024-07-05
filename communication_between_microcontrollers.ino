#include <HardwareSerial.h>

HardwareSerial mySerial(1);

void setup() {
  Serial.begin(115200); // パソコンへのデバッグ出力用
  mySerial.begin(9600, SERIAL_8N1, 3, 1); // RX=16, TX=17 (ピン番号は適宜変更してください)
  Serial.println("ESP32 started");
}

void loop() {
  // ESP32側で受信データを処理
  if (mySerial.available()) {
    Serial.print("unko");
    char c = mySerial.read();
    Serial.print("Received from Raspberry Pi: ");
    Serial.println(c); // 受信データをシリアルモニタに表示
  }

  // ESP32側で送信データを処理
  if (Serial.available()) {
    char c = Serial.read();
    Serial.print("Sending to Raspberry Pi: ");
    Serial.println(c); // 送信データをシリアルモニタに表示
    mySerial.write(c); // パソコンからの入力をRaspberry Piに送信
  }
}
