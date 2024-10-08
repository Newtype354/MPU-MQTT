#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <MPU6050.h> 

MPU6050 mpu;

// connect ke wifi terdekat
const char* ssid = "Nahami_21";
const char* password = "Nahami_21";
const char* mqtt_server = "52.230.104.212";  // Broker MQTT yang ingin dipakai

WiFiClient espClient;
PubSubClient client(espClient);

// Kalman filter variables
double Q_angle = 0.001;  // Process noise variance for the accelerometer
double Q_bias = 0.003;   // Process noise variance for the gyro bias
double R_measure = 0.03; // Measurement noise variance

double angleX = 0.0;  // The angle for X-axis calculated by the Kalman filter
double angleY = 0.0;  // The angle for Y-axis calculated by the Kalman filter
double biasX = 0.0;   // The gyro bias calculated by the Kalman filter for X
double biasY = 0.0;   // The gyro bias calculated by the Kalman filter for Y
double rateX, rateY;  // Unbiased rate calculated from the gyro

double P[2][2] = { { 0, 0 }, { 0, 0 } }; // Error covariance matrix for X
double P1[2][2] = { { 0, 0 }, { 0, 0 } }; // Error covariance matrix for Y

unsigned long timer;
double accX, accY, accZ;
double gyroX, gyroY;
double dt;

// Kalman Filter function for X and Y axis
double KalmanFilterX(double newAngle, double newRate, double dt) {
  rateX = newRate - biasX;
  angleX += dt * rateX;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  double S = P[0][0] + R_measure;
  double K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  double y = newAngle - angleX;
  angleX += K[0] * y;
  biasX += K[1] * y;

  double P00_temp = P[0][0];
  double P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angleX;
}

double KalmanFilterY(double newAngle, double newRate, double dt) {
  rateY = newRate - biasY;
  angleY += dt * rateY;

  P1[0][0] += dt * (dt*P1[1][1] - P1[0][1] - P1[1][0] + Q_angle);
  P1[0][1] -= dt * P1[1][1];
  P1[1][0] -= dt * P1[1][1];
  P1[1][1] += Q_bias * dt;

  double S = P1[0][0] + R_measure;
  double K[2];
  K[0] = P1[0][0] / S;
  K[1] = P1[1][0] / S;

  double y = newAngle - angleY;
  angleY += K[0] * y;
  biasY += K[1] * y;

  double P00_temp = P1[0][0];
  double P01_temp = P1[0][1];

  P1[0][0] -= K[0] * P00_temp;
  P1[0][1] -= K[0] * P01_temp;
  P1[1][0] -= K[1] * P00_temp;
  P1[1][1] -= K[1] * P01_temp;

  return angleY;
}

// nyambung wifi ke WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Fungsi untuk terhubung ke MQTT server
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  // Inisialisasi komunikasi serial
  Serial.begin(9600);

  setup_wifi(); // Menghubungkan ke WiFi
  client.setServer(mqtt_server, 1883); // Menghubungkan ke broker MQTT

  // Inisialisasi I2C
  Wire.begin();
  
  // Inisialisasi MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Get initial sensor readings
  accX = mpu.getAccelerationX();
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();

  // Set initial time
  timer = micros();
}

void loop() {
  if (!client.connected()) {
    reconnect(); // Menghubungkan kembali jika koneksi MQTT terputus
  }
  client.loop();

  // Membaca data accelerometer
  accX = mpu.getAccelerationX();
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();

  // Membaca data gyroscope
  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();

  // Calculate delta time
  dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  // Menghitung sudut pitch dan roll dari accelerometer
  double pitch = atan(accX / sqrt(accY * accY + accZ * accZ)) * 180 / PI;
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * 180 / PI;

  // Apply Kalman filter
  double kalmanPitch = KalmanFilterX(pitch, gyroX / 131.0, dt);
  double kalmanRoll = KalmanFilterY(roll, gyroY / 131.0, dt);

  // Mengirimkan data melalui MQTT
  String payload = "KP (atas bawah): " + String(kalmanPitch) + ", KR (miring kiri kanan): " + String(kalmanRoll);
  client.publish("mpuku", payload.c_str());

  // Menampilkan data accelerometer, gyroscope, dan hasil Kalman filter di Serial Monitor
  Serial.println(payload);

  delay(100);
}
