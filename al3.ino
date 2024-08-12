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
#define EARTH_RAD 6378137 // m


Adafruit_BNO055 bno=Adafruit_BNO055(55, 0x28);;
const int BME_CS = 5;
const int BME_SCK = 18;
const int BME_MOSI = 23;
const int BME_MISO = 19;
//グローバル関数（雑）
double oira_data[3] = {0, 0, 0};
double pres_data[3] = {0, 0, 0};  
double acc_data[3] = {0, 0, 0};  
double gps_data[2] = {0,0};
double mag_data[2] ={0,0};
double leg_data[2] = {0,0};
double gps_first[2] = {0,0};
double kas_data[2] = {0,0};
double acc2_data[2] = {0,0};
double mag_offset = 0;
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);
TinyGPSPlus gps;
double gps_lat; // 緯度
double gps_lon; // 経度
//モーター系
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
//pid_gps
#define gpsSp 255
#define gpsTp 1
#define gpsSps 235

float last_turn_error = 0.0;
float last_d_term = 0.0;
float alpha = 0.1;
float kp = 0.5;
float kd = 0;
float ki = 0;
float last_turn = 0;
float i_turn = 0;
float integral_turn = 0.0;
int PID_left = 0;
int PID_right = 0;
//pid_cam
float last_turn_errors = 0.0;
float last_d_terms = 0.0;
float alphas = 0.1;
float kps = 0.5;
float kds = 0;
float kis = 0;
float last_turns = 0;
float i_turns = 0;
float integral_turns = 0.0;
int PID_lefts = 0;
int PID_rights = 0;

//setup
void setup() {
  Serial.begin(115200); // シリアル通信の初期化
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!Serial);


  pinMode(21, INPUT_PULLUP); //SDA 21番ピンのプルアップ
  pinMode(22, INPUT_PULLUP); //SDA 22番ピンのプルアップ
  pinMode(32, OUTPUT); //溶断
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
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

}

//前進
void mae(int i,int j){
  //Serial.println("Motor A Forward");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  //Serial.println("Motor B Forward");
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(LEDC_CHANNEL_A, i);
  ledcWrite(LEDC_CHANNEL_B, j);
}


//停止
void stop(){
  //Serial.println("Stop");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW); 

}

// 回転_右
void spin_r(int i, int j){
  //Serial.println("spin_r");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(LEDC_CHANNEL_A, i);
  ledcWrite(LEDC_CHANNEL_B, j);
}

// 回転_左
void spin_l(int i, int j){
  //Serial.println("spin_l");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(LEDC_CHANNEL_A, i);
  ledcWrite(LEDC_CHANNEL_B, j);
}

//後退
void back(int i, int j){
  //Serial.println("back");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(LEDC_CHANNEL_A, i);
  ledcWrite(LEDC_CHANNEL_B, j);  
}

// オイラ角取得
float get_oira() {
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


  double yaws = yaw * 57.2957795131;    // ラジアンから度に変換

  return(yaws);

}

//地磁気とる
void get_mag() {
  imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  /*
  Serial.print(" Mg_xyz:");
  Serial.print(magnetmetor .x());
  Serial.print(", ");
  Serial.print(magnetmetor .y());
  Serial.print(", ");
  Serial.print(magnetmetor .z());
  */
  float heading = calculate_heading(magnetmetor.x(), magnetmetor.y());
  /*
  Serial.print(" Heading: ");
  Serial.println(heading);
  */

}



//地磁気からの方位角
float calculate_heading(float x, float y) {
  // アークタンジェントで方位角を計算（ラジアンから度に変換）
  float heading = atan2(y, x) * 180 / PI;


  if (heading < 0) {
    heading += 360;
  }
  mag_data[0] = heading;

  return heading;
}

//めんどいから分けた
void get_mag2() {
  imu::Vector<3> magnetmetor = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  /*
  Serial.print(" Mg_xyz:");
  Serial.print(magnetmetor .x());
  Serial.print(", ");
  Serial.print(magnetmetor .y());
  Serial.print(", ");
  Serial.print(magnetmetor .z());
  */
  float heading = calculate_heading2(magnetmetor.x(), magnetmetor.y());
  
  Serial.print(" Heading: ");
  Serial.println(heading);
}

//上に同じ
float calculate_heading2(float x, float y) {
  // アークタンジェントで方位角を計算（ラジアンから度に変換）
  float heading = atan2(y, x) * 180 / PI;

  if (heading < 0) {
    heading += 360;
  }
  mag_data[1] = heading;

  return heading;
}

//加速度取得
void get_kas(){
  imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print(" Ac_xyz:");
  Serial.print(accelermetor.x());
  Serial.print(", ");
  Serial.print(accelermetor.y());
  Serial.print(", ");
  Serial.print(accelermetor.z());
  kas_data[0]=accelermetor.z();
}

// ジャイロ角速度取得
void get_acc() {
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  double gx = gyroscope.x();
  double gy = gyroscope.y();
  double gz = gyroscope.z();

  Serial.print("acc");
  Serial.print(gyroscope.z());
  Serial.println(gz);
  int s;
  int i;
  for(s=0;s<3;s++){
    float acc=0;
    for (i=0;i<5;i++){
      acc = acc + gz;
      Serial.println(gz);
      Serial.println(acc);
      delay(1000);
    }
    acc_data[s] = acc/5;
  }
  Serial.println(acc_data[0]);
}

void get_acc2() {
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  double gx = gyroscope.x();
  double gy = gyroscope.y();
  double gz = gyroscope.z();
  Serial.println(gx);
  Serial.println(gy);
  Serial.println(gz);
  acc2_data[0]=gx;
  acc2_data[1]=gy;
  acc2_data[2]=gz;
}

// 気圧取得
void get_pres() {
  int s=0;
  float pres = bme.readPressure() / 100.0;
  for(s=0;s<3;s++){
  float pres = bme.readPressure() / 100.0;
  Serial.print("Pressure: ");
  Serial.print(pres);
  Serial.println(" hPa");
  pres_data[s] = pres;
  delay(1000);
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
    pres2 = pres_data[2] - pres_data[1];

    if (pres1 > 0.1 && pres2 > 0.1) {
      delay(1000);
    } 
    else {
      break;
    }
  }
  Serial.println("Reached the top");
}

// 放出判定
void left() {
  while (1) {
    get_oira();
    Serial.println(oira_data[0]);
    if (oira_data[0] > 35 || oira_data[0] < -35) {
      delay(1000);
    } 
    else {
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
    if (pres1 < 0.03 && pres2 < 0.03) {
      Serial.println("pres_ok");
      while (1) {
        get_acc();
        double acc1 = acc_data[1] - acc_data[0];
        double acc2 = acc_data[2] - acc_data[1];
        if (abs(acc1) > 0.5 && abs(acc2) > 0.5) {
          delay(5000);
        } 
        else {
          Serial.println("acc_ok");
          break;
        }
      }
      break;
    }
  }
  Serial.println("cyakuchi");
}

//溶断
void yodan(){
  digitalWrite(32, HIGH);
  delay(200);
  digitalWrite(32,LOW);
  delay(200);
  Serial.println("yodan");
}

//スタック判定
void stack(){
  get_acc2();
  if(abs(acc2_data[0]) + abs(acc2_data[1]) + abs(acc2_data[2]) < 6){
    Serial.println("stack");
    spin_l(255,255);
    delay(2000);
    mae(255,255);
    delay(3000);
  }
  else{
    Serial.println("not_stack");
  }
}

//スタック判定2
void stack2(){
  get_kas();
  if(kas_data[0] < 0){
    Serial.println("stack");
    spin_l(255,255);
    delay(2000);
    mae(255,255);
    delay(2000);
  }
  else{
    Serial.println("not_stack");
  }
}

//パラ回避
void para() {
  Serial.println("par_sta");
    while (Serial.available()) {
        String command = Serial.readStringUntil('\n'); // 改行まで読み取る

        // コマンドに応じた処理を実行
        if (command == "parar") {
          Serial.println("para");
          spin_l(230,230);
          delay(500);
        }
        else if (command == "paral") {
          Serial.println("para");
          spin_r(230,230);
          delay(500);
        }
        else if (command == "non") {
            Serial.println("non para");
            mae(255,255);
            delay(3000);
            break;
        } 
    }
}

//位相角計算
double Azimuth(double lat1, double lng1, double lat2, double lng2) {
  double x1 = lng1 * M_PI / 180.0;
  double y1 = lat1 * M_PI / 180.0;
  double x2 = lng2 * M_PI / 180.0;
  double y2 = lat2 * M_PI / 180.0;
  double x_dif = x2 - x1;
  double azimuth = atan2(sin(x_dif), (cos(y1) * tan(y2) - sin(y1) * cos(x_dif))) * 180 / M_PI;

  return azimuth;
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

//gpsのデータ取得
void gp_data() {
  //Serial.println("made");
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        gps.encode(c);
        if (gps.location.isUpdated()) {
            gps_lat = gps.location.lat();
            gps_lon = gps.location.lng();
            /*
            Serial.print("LAT:  ");
            Serial.println(gps_lat, 9);
            Serial.print("LON:  ");
            Serial.println(gps_lon, 9);
            */
            
           // ゴール地点の緯度,経度
            double goal_lat = 35.924685250;// ゴールの緯度
            double goal_lon = 139.911880667; // ゴールの経度

            // 現在地からゴール地点への方位角の計算
            double azimuth = Azimuth(gps_lat, gps_lon, goal_lat, goal_lon);
            // 現在地からゴール地点までの距離の計算
            double distance = calculate_distance(gps_lat, gps_lon, goal_lat, goal_lon);
            
            Serial.print("方位角: ");
            Serial.println(azimuth);
            Serial.print("距離: ");
            Serial.print(distance);
            Serial.println(" m");
            
            gps_data[0] = azimuth;   
            gps_data[1] = distance;
            break;
          }
     }
}

//角度の差
double get_turn() {
  gp_data();
  get_mag();
  double currentmag = mag_data[0]; 
  double goalDirection = gps_data[0];
  double relativeDirection = goalDirection - currentmag;
  Serial.print("currentmag:");
  Serial.println(currentmag);
  Serial.print("goalDirection:");
  Serial.println(goalDirection);
  Serial.print("relativeDirection:");
  Serial.println(relativeDirection);
  if (relativeDirection < -180) {
    relativeDirection += 360;
  } else if (relativeDirection > 180) {
    relativeDirection -= 360;
  }
  stack2();

  leg_data[0] = relativeDirection;
  return relativeDirection;
}

//ゴール方向に機体を向ける
void adjust() {
  while (true) {
    get_turn();
    /*
    Serial.println("finish get_turn");
    */
    double ave_turn = leg_data[0];
    double ave_dit = gps_data[1];
    Serial.println(ave_turn);
    
    if (abs(ave_turn) <= 20) {
      stop();
      Serial.println("ok_adjust");
      break;
    } 
    else if(ave_turn < 0){
        spin_r(200, 200);
        delay(300);        
      }
    else if(ave_turn > 0){
        spin_l(200, 200);
        delay(300);         
      }
    }
}

//gps誘導
void pid() {
  while (true) {
    get_turn();
    stack();
    stack2();

    double ave_turn = leg_data[0];
    double ave_dit = gps_data[1];

    if (ave_dit < 5) {
        stop();
        Serial.println("ok_pid");
        break;
    } 
    else {
        float turn_error = ave_turn;

        float p_term = kp * turn_error;
        integral_turn += (turn_error + last_turn_error) / 2;
     
        float i_term = ki * integral_turn;
        float d_term_raw = turn_error - last_turn_error;
        float d_term = alpha * d_term_raw + (1 - alpha) * last_d_term;
        last_d_term = d_term;

        float pid_output = p_term + i_term + d_term;

        float last_turn_error = turn_error;

        if (pid_output > 0) {
            PID_left = gpsSp;
            PID_right = gpsSp - pid_output;
        } 
        else if (pid_output < 0) {
            PID_right = gpsSp;
            PID_left = gpsSp + pid_output;
        }
        else if (pid_output = 0) {
            PID_right = 255;
            PID_left = 255;
        }        
        
        PID_left = constrain(PID_left, 200, 255);
        PID_right = constrain(PID_right, 200, 255);
        Serial.print("\tp_term :");
        Serial.print(p_term);
        Serial.print("\tpid_output :");
        Serial.print(pid_output);
        Serial.print("\tMoterpower : ");
        Serial.print(PID_left);
        Serial.print(",");
        Serial.println(PID_right);
        
        mae(PID_right, PID_left);
        delay(1000);
        
    }
 }
}

//カメラによるゴールの方角調整
void camera_yudou(){
  Serial.println("start_cam");
  while(true){
      if (Serial.available()>0) {
        String command = Serial.readStringUntil('\n'); // 改行まで読み取る

 //rasへのコマンド指令
        if (command == "centor") {
          stop();
          Serial.println("ok");
          break;

        }
        else if (command == "right") {
          spin_l(200,200);
          delay(100);
          stop();
        } 
        else if (command == "left") {
          spin_r(200,200);
          delay(100);
          stop();
        }
        else{
          spin_l(200,200);
          delay(1000);
          stop();
        }

    }
  }
}

//めんどいから分けた
double get_turn2() {
  double currentmag = get_oira(); 
  double goalDirection = oira_data[2];
  double relativeDirection = goalDirection - currentmag;
  
  Serial.println(currentmag);
  Serial.println(goalDirection);
  Serial.println(relativeDirection);
  if (relativeDirection < -180) {
    relativeDirection += 360;
  } 
  else if (relativeDirection > 180) {
    relativeDirection -= 360;
  }

  leg_data[1] = relativeDirection;
  return relativeDirection;
}

//画像でのゴール誘導（占有率）
void goal() {
    double yaws = get_oira();
    oira_data[2] = yaws;
    Serial.print("yaw:");
    Serial.println(oira_data[2]);
    Serial.println("start_goal"); //rasへのコマンド指令
    delay(5000);
    while(true){
      if (Serial.available()>0) {
        String command = Serial.readStringUntil('\n'); // 改行まで読み取る
        get_turn2();
        stack();
        stack2();
        double ave_turns = leg_data[1];
        Serial.println(ave_turns);

        if (command == "arrival") {
          stop();
          Serial.println("goal");
          Serial.println("end_cam");
          break;
        }
        else if(command == "zensin"){
          float turn_errors = ave_turns;

          float p_terms = kps * turn_errors;
          integral_turns += (turn_errors + last_turn_error) / 2;
      
          float i_terms = ki * integral_turns;
          float d_term_raw = turn_errors - last_turn_error;
          float d_term = alpha * d_term_raw + (1 - alpha) * last_d_term;
          last_d_term = d_term;

          float pid_outputs = p_terms + i_terms + d_term;

          float last_turn_errors = turn_errors;

          if (pid_outputs > 0) {
              PID_lefts = gpsSps;
              PID_rights = gpsSps - pid_outputs;
          } 
          else if (pid_outputs < 0) {
              PID_rights = gpsSps;
              PID_lefts = gpsSps + pid_outputs;
          }
          else if (pid_outputs = 0) {
              PID_rights = 235;
              PID_lefts = 235;
          }        
          
          PID_lefts = constrain(PID_lefts, 200, 235);
          PID_rights = constrain(PID_rights, 200, 235);
          Serial.print("\tp_term :");
          Serial.print(p_terms);
          Serial.print("\tpid_output :");
          Serial.print(pid_outputs);
          Serial.print("\tMoterpower : ");
          Serial.print(PID_lefts);
          Serial.print(",");
          Serial.println(PID_rights);
          
          mae(PID_lefts, PID_rights);
          delay(500);
        }
    }
  }
}

//キャリア脱出
void escape() {
  mae(250,250);
  delay(3000);
  stop();
  stack();
  stack2();
  Serial.println("escape_from_career");
}

//起動のための
void first() {
  while(true){
    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
  
      if (command == "start") {
        Serial.println("begin");
        break;
      }

    }
  }
}

void loop(){
  level();//収納判定
  upper();//浮上判定
  left();//放出判定
  land();//着地判定
  first();//ラズパイ側で何か入力
  level();//収納判定
  upper();//浮上判定
  left();//放出判定
  land();//着地判定
  yodan();//溶断
  escape();//キャリア脱出
  adjust();//角度調整
  para();//パラ回避
  Serial.println("next_pid");
  pid();//gps誘導
  Serial.println("next_camera");
  camera_yudou();//画像角度調整
  Serial.println("next_goal");
  goal();//画像誘導ゴール判定
  Serial.println("finish!!");
}