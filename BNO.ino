#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <Ticker.h>
#include <cmath>

Adafruit_BME280 bme;
Adafruit_BNO055 bno=Adafruit_BNO055(55, 0x28);;
const int BME_CS = 5;
const int BME_SCK = 18;
const int BME_MOSI = 23;
const int BME_MISO = 19;
double oira_data[3] = {0, 0, 0};
double pres_data[3] = {0, 0, 0};  
double acc_data[3] = {0, 0, 0};    
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
//setup
void setup() {
  Serial.begin(115200); // シリアル通信の初期化
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
  oira_data[2] = yaw * 57.2957795131;    // ラジアンから度に変換

  Serial.println(oira_data[0]);
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

// 収納判定
void level() {
  
  while (1) {
    get_oira();
    if (oira_data[0] > -50 && oira_data[0] <50) {
      delay(10000);
    } 
    else {
      break;
    }
  }
  Serial.println("In the tube");
}

// 浮上判定
void upper() {
  double pres1 = 0;
  double pres2 = 0;
  while (1) {
    get_pres();
    pres1 = pres_data[1] - pres_data[0];
    pres2 = pres_data[2] - pres_data[0];
    if (pres1 < 1 && pres2 < 1) {
      delay(1000);
    } else {
      break;
    }
  }
  Serial.println("Reached the top");
}

// 放出判定
void left() {
  while (1) {
    get_oira();
    if (oira_data[0] > 40 || oira_data[0] < -40) {
      delay(1000);
    } else {
      break;
    }
  }
  Serial.println("Left the tube");
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

void loop() {
  level();
  upper();
  left();
  land();
}

