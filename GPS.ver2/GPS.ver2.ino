#include <TinyGPS++.h>
#include <math.h>

TinyGPSPlus gps;

double gps_lat; // 緯度
double gps_lon; // 経度

#define PI 3.14159265
#define EARTH_RAD 6378.137 // km

// 方位角計算
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

void setup() {
    // シリアルポート開始
    Serial.begin(115200);
    Serial2.begin(9600); // NEO-7M デフォルト
}

void loop() {
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

            // 方位角と距離を出力
            Serial.print("方位角: ");
            Serial.println(bearing);
            Serial.print("距離: ");
            Serial.print(distance);
            Serial.println(" km");

            delay(5000); // 5秒待機
        }
    }
}

