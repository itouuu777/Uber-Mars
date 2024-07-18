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

// オイラ角取得
void get_oira() {
  imu::Quaternion quat = bno.getQuat();

  double w = quat.w();
  double x = quat.x();
  double y = quat.y();
  double z = quat.z();

  // ロール (x軸の回転)
  double t0 = +2.0 * (w * x + y * z);
  double t1 = +1.0 - 2.0 * (x * x + y * y);
  double roll = atan2(t0, t1);

  // ピッチ (y軸の回転)
  double t2 = +2.0 * (w * y - z * x);
  double pitch = asin(t2);

  // ヨー (z軸の回転)
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (y * y + z * z);
  double yaw = atan2(t3, t4);

  oira_data[0] = roll * 57.2957795131;   // ラジアンから度に変換
  oira_data[1] = pitch * 57.2957795131;  // ラジアンから度に変換
  oira_data[2] = yaw * 57.2957795131 + yaw_offset;    // ラジアンから度に変換

  Serial.println(oira_data[0]);
}

// 気圧取得
void get_pres() {
  float pres = bme.readPressure() / 100.0;
  int s;
  int i;
  for(s=0;s<3;s++){
    float kiat=0;
    for (i=0;i<5;i++){
      kiat = kiat + pres;
      delay(1000);
    }
    pres_data[s] = kiat/5;
  }
}

// ジャイロ角速度取得
void get_acc() {
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  double gx = gyroscope.x();
  double gy = gyroscope.y();
  double gz = gyroscope.z();
  int s;
  int i;
  for(s=0;s<3;s++){
    float acc=0;
    for (i=0;i<5;i++){
      acc = acc + gz;
      delay(1000);
    }
    acc_data[s] = acc/5;
  }
}

// 着地判定
void land() {
  double pres1 = 0;
  double pres2 = 0;
  while (1) {
    get_pres();
    pres1 = pres_data[1] - pres_data[0];
    pres2 = pres_data[2] - pres_data[1];
    if (pres1 < 1 && pres2 < 1) {
      while (1) {
        get_acc();
        double acc1 = acc_data[1] - acc_data[0];
        double acc2 = acc_data[2] - acc_data[1];
        if (acc1 < 1 && acc2 < 1) {
          delay(1000);
        } else {
          break;
        }
      }
      break;
    }
  }
  Serial.println("Touch down");
}

//溶断
void yodan(){

  digitalWrite(32, HIGH);
  delay(500);
  digitalWrite(32,LOW);
  delay(500);
  Serial.println("yodan");
    
}

double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double delta_lon = lon2 - lon1;
    double x = sin(delta_lon * PI / 180.0) * cos(lat2 * PI / 180.0);
    double y = cos(lat1 * PI / 180.0) * sin(lat2 * PI / 180.0) - sin(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * cos(delta_lon * PI / 180.0);
    double bearing_rad = atan2(x, y);
    double bearing_deg = bearing_rad * (180.0 / PI);
    return fmod((bearing_deg + 360), 360); // 方位角を0-360の範囲内に調整
}

// 度をラジアンに変換
double deg2rad(double deg) {
    return deg * PI / 180.0;
}

// 距離計算
double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    lat1 = deg2rad(lat1);
    lon1 = deg2rad(lon1);
    lat2 = deg2rad(lat2);
    lon2 = deg2rad(lon2);

    return EARTH_RAD * acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lon2 - lon1));
}

void gp_data() {
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        gps.encode(c);
        if (gps.location.isUpdated()) {
            gps_lat = gps.location.lat();
            gps_lon = gps.location.lng();
            Serial.print("LAT:  ");
            Serial.println(gps_lat, 9);
            Serial.print("LON:  ");
            Serial.println(gps_lon, 9);

            // ゴール地点の緯度,経度
            double goal_lat = 35.918094635;  // ゴールの緯度
            double goal_lon = 139.908639371; // ゴールの経度

            // 現在地からゴール地点への方位角の計算
            double bearing = calculate_bearing(gps_lat, gps_lon, goal_lat, goal_lon);

            // 現在地からゴール地点までの距離の計算
            double distance = calculate_distance(gps_lat, gps_lon, goal_lat, goal_lon);
            gps_data[0] = bearing;   
            gps_data[1] = distance;
          }
    }

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

            for(int i = 230; i >= 180; i --){
              ledcWrite(LEDC_CHANNEL_A, 230);
              ledcWrite(LEDC_CHANNEL_B, i);
              delay(10);
              Serial.print("Speed A: ");
              Serial.print(i);
              Serial.print(" | Speed B: ");
              Serial.println(i);
            } 
            for(int i = 180; i <= 230; i ++){
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

void loop() {
  para();
  land();
  yodan();
  get_pres();
  delay(5000);
  gp_data();
  delay(5000);
  get_oira();
  delay(1000);
  para();
}
