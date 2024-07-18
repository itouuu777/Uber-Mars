#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <Ticker.h>
#include <cmath>
#include <TinyGPS++.h>
#include <math.h>
#include <HardwareSerial.h>
#define RXD2 16
#define TXD2 17
#define PI 3.14159265
#define EARTH_RAD 6378.137 // km


Adafruit_BNO055 bno=Adafruit_BNO055(55, 0x28);;
const int BME_CS = 5;
const int BME_SCK = 18;
const int BME_MOSI = 23;
const int BME_MISO = 19;
double oira_data[3] = {0, 0, 0};
double pres_data[3] = {0, 0, 0};  
double acc_data[3] = {0, 0, 0};  
double gps_data[2] = {0,0};
double yaw_offset = 17.8;
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
TinyGPSPlus gps;
double gps_lat; // 緯度
double gps_lon; // 経度
const int PWMA = 12;
const int AIN1 = 13;
const int AIN2 = 14;
const int STBY = 25;
const int PWMB = 2;
const int BIN1 = 4;
const int BIN2 = 0;
const int LEDC_CHANNEL_A = 0;
const int LEDC_CHANNEL_B = 1;
const int LEDC_BASE_FREQ = 1000;
double relativeDirection;
//pid
#define gpsSp 200
#define gpsTp 0.75

float last_turn_error = 0.0;
float last_d_term = 0.0;
float alpha = 0.1;
float kp = 0.8;
float kd = 0;
float ki = 0.1;
float last_turn = 0;
float i_turn = 0;
float integral_turn = 0.0;
int PID_left;
int PID_right;
//setup
void setup() {
  Serial.begin(115200); // シリアル通信の初期化
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!Serial);


  pinMode(21, INPUT_PULLUP); //SDA 21番ピンのプルアップ
  pinMode(22, INPUT_PULLUP); //SDA 22番ピンのプルアップ

  Serial.println("Orientation Sensor Raw Data Test");

  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");

  bno.setExtCrystalUse(false);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

    // PWMの初期化
  Serial.println("Initializing PWM for motor A...");
  ledcSetup(LEDC_CHANNEL_A, LEDC_BASE_FREQ, 8);
  ledcAttachPin(PWMA, LEDC_CHANNEL_A);

  Serial.println("Initializing PWM for motor B...");
  ledcSetup(LEDC_CHANNEL_B, LEDC_BASE_FREQ, 8);
  ledcAttachPin(PWMB, LEDC_CHANNEL_B);

  // GPIOピンの初期化
  Serial.println("Initializing GPIO pins...");
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  Serial.println("Setting STBY to HIGH...");
  digitalWrite(STBY, HIGH); // スタンバイ解除
  

}

void mae(int i,int j){
  Serial.println("Motor A Forward");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  Serial.println("Motor B Forward");
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(LEDC_CHANNEL_A, i);
  ledcWrite(LEDC_CHANNEL_B, j);
}


// 停止
void stop(){
  Serial.println("Stop");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH); // AIN1とAIN2を同時にHIGHにしてブレーキ状態にする

}




void para() {
    while (Serial.available()) {
        String command = Serial.readStringUntil('\n'); // 改行まで読み取る

        Serial.println("par_sta");

        // コマンドに応じた処理を実行
        if (command == "para") {
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);

            for(int i = 230; i <= 180; i--){
              ledcWrite(LEDC_CHANNEL_A, 230);
              ledcWrite(LEDC_CHANNEL_B, i);
              delay(10);
              Serial.print("Speed A: ");
              Serial.print(i);
              Serial.print(" | Speed B: ");
              Serial.println(i);
            } 
            for(int i = 180; i <= 230; i--){
              ledcWrite(LEDC_CHANNEL_A, 230);
              ledcWrite(LEDC_CHANNEL_B, i);
              delay(10);
              Serial.print("Speed A: ");
              Serial.print(i);
              Serial.print(" | Speed B: ");
              Serial.println(i);
            } 
          Serial.println("kaihi");
          Serial.println("end_cam");
          adjust();
        }

         
        else if (command == "non") {
            Serial.println("non para");
        } 
        else {
            Serial.println("Unknown command");
        }
    }
}

void(){
  mae(200,200);
  delay(30000);
  //30秒間
  para();
}