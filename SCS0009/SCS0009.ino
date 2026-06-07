#include <M5Atom.h>
#include <Kalman.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PS4Controller.h>

#define servoL 1
#define servoR 2


WebServer server(80);
const char ssid[] = "AtomMotionSCS";  // SSID (任意)
const char pass[] = "password";   // password (任意)
const IPAddress ip(192, 168, 55, 92);      //　IP address (任意)
const IPAddress subnet(255, 255, 255, 0);


unsigned long oldTime = 0, loopTime, nowTime;
float dt;


float Kp = 1.5, Kd = 3.0 ,Kw = 1.5;
float Ki = 1.0;
float Kyaw = 0.15;
float Kvel = 0.6; 
float Kroll = 2.5;
int Delay = 0;


int GetUP = 0;
float M, ML1, MR1;

float theta0_hat = 0.0;
float vel = 0.0;
float Tturn = 0.0;
float omegaCmd = 0.0, velCmd =0.0, velCmdFiler =0.0;


float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float theta_X = 0.0, theta_Y = 0.0;
float theta_Xdot = 0.0, theta_Ydot = 0.0, theta_Zdot = 0.0;

Kalman kalmanX, kalmanY;
float kalAngleX, kalAngleDotX, kalAngleY, kalAngleDotY;

Preferences preferences;


//加速度センサから傾きデータ取得 [deg]
void get_theta() {
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  //傾斜角導出 単位はdeg
  theta_X =  atan2(accY, -accZ) * RAD_TO_DEG;
  theta_Y  = -atan2(-accX, -accZ) * RAD_TO_DEG;
}

//角速度取得
void get_gyro() {
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  theta_Xdot = -gyroX;
  theta_Ydot = gyroY;
  theta_Zdot = -gyroZ;
}


//https://qiita.com/Ninagawa123/items/7b79c5f5117dd1470ac9
//コチラを参考に Time parameterを設定するように修正
void scs_setSpeed(byte id, int speedSigned) {
  uint16_t speed;
  if (speedSigned >= 0) {
    speed = constrain(speedSigned, 0, 1000);
  } else {
    speed = constrain(-speedSigned, 0, 1000) | 0x0400;
  }

  byte message[13];
  message[0] = 0xFF;
  message[1] = 0xFF;
  message[2] = id;
  message[3] = 9;
  message[4] = 3;
  message[5] = 42;
  message[6] = 0x00;
  message[7] = 0x00;
  message[8] = (speed >> 8) & 0xFF;
  message[9] = speed & 0xFF;
  message[10] = 0x00;
  message[11] = 0x00;

  byte checksum = 0;
  for (int i = 2; i < 12; i++) {
    checksum += message[i];
  }

  message[12] = ~checksum;

  for (int i = 0; i < 13; i++) {
    Serial1.write(message[i]);
  }
}


//ブラウザ表示
void handleRoot() {
  String temp ="<!DOCTYPE html> \n<html lang=\"ja\">";
  temp +="<head>";
  temp +="<meta charset=\"utf-8\">";
  temp +="<title>AtomMotionSCS</title>";
  temp +="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  temp +="<style>";
  temp +=".container{";
  temp +="  max-width: 500px;";
  temp +="  margin: auto;";
  temp +="  text-align: center;";
  temp +="  font-size: 1.2rem;";
  temp +="}";
  temp +="span,.pm{";
  temp +="  display: inline-block;";
  temp +="  border: 1px solid #ccc;";
  temp +="  width: 50px;";
  temp +="  height: 30px;";
  temp +="  vertical-align: middle;";
  temp +="  margin-bottom: 20px;";
  temp +="}";
  temp +="span{";
  temp +="  width: 120px;";
  temp +="}";
  temp +="button{";
  temp +="  width: 100px;";
  temp +="  height: 40px;";
  temp +="  font-weight: bold;";
  temp +="  margin-bottom: 20px;";
  temp +="}";
  temp +="button.on{ background:lime; color:white; }";
  temp +=".column-2{ max-width:250px; margin:auto; text-align:center; display:flex; justify-content:space-between; flex-wrap:wrap; }";
  temp +="</style>";
  temp +="</head>";
  
  temp +="<body>";
  temp +="<div class=\"container\">";
  

  
  //Kp
  temp +="Kp<br>";
  temp +="<a class=\"pm\" href=\"/KpM\">-</a>";
  temp +="<span>" + String(Kp) + "</span>";
  temp +="<a class=\"pm\" href=\"/KpP\">+</a><br>";

  //Kd
  temp +="Kd<br>";
  temp +="<a class=\"pm\" href=\"/KdM\">-</a>";
  temp +="<span>" + String(Kd) + "</span>";
  temp +="<a class=\"pm\" href=\"/KdP\">+</a><br>";

  //Kw
  temp +="Kw<br>";
  temp +="<a class=\"pm\" href=\"/KwM\">-</a>";
  temp +="<span>" + String(Kw) + "</span>";
  temp +="<a class=\"pm\" href=\"/KwP\">+</a><br>";

  //Ki
  temp +="Ki<br>";
  temp +="<a class=\"pm\" href=\"/KiM\">-</a>";
  temp +="<span>" + String(Ki) + "</span>";
  temp +="<a class=\"pm\" href=\"/KiP\">+</a><br>";

  //Kyaw
  temp +="Kyaw<br>";
  temp +="<a class=\"pm\" href=\"/KyawM\">-</a>";
  temp +="<span>" + String(Kyaw) + "</span>";
  temp +="<a class=\"pm\" href=\"/KyawP\">+</a><br>";

  //Kvel
  temp +="Kvel<br>";
  temp +="<a class=\"pm\" href=\"/KvelM\">-</a>";
  temp +="<span>" + String(Kvel) + "</span>";
  temp +="<a class=\"pm\" href=\"/KvelP\">+</a><br>";

  //Kroll
  temp +="Kroll<br>";
  temp +="<a class=\"pm\" href=\"/KrollM\">-</a>";
  temp +="<span>" + String(Kroll) + "</span>";
  temp +="<a class=\"pm\" href=\"/KrollP\">+</a><br>";

  //Delay
  temp +="Delay<br>";
  temp +="<a class=\"pm\" href=\"/DelayM\">-</a>";
  temp +="<span>" + String(Delay) + "</span>";
  temp +="<a class=\"pm\" href=\"/DelayP\">+</a><br>";
  
  
  temp +="</div>";
  temp +="</body>";
  server.send(200, "text/HTML", temp);
}


void KpM() {
  if(Kp >= -300.0){
    Kp -= 0.1;
    preferences.putFloat("Kp", Kp);
  }
  handleRoot();
}
void KpP() {
  if(Kp <= 300){
    Kp += 0.1;
    preferences.putFloat("Kp", Kp);
  }
  handleRoot();
}

void KdM() {
  if(Kd >= -300.0){
    Kd -= 0.1;
    preferences.putFloat("Kd", Kd);
  }
  handleRoot();
}
void KdP() {
  if(Kd <= 300){
    Kd += 0.1;
    preferences.putFloat("Kd", Kd);
  }
  handleRoot();
}

void KwM() {
  if(Kw >= -30){
    Kw -= 0.1;
    preferences.putFloat("Kw", Kw);
  }
  handleRoot();
}
void KwP() {
  if(Kw <= 30){
    Kw += 0.1;
    preferences.putFloat("Kw", Kw);
  }
  handleRoot();
}

void KiM() {
  if(Ki >= -10){
    Ki -= 0.1;
    preferences.putFloat("Ki", Ki);
  }
  handleRoot();
}
void KiP() {
  if(Ki <= 10){
    Ki += 0.1;
    preferences.putFloat("Ki", Ki);
  }
  handleRoot();
}

void KvelM() {
  if(Kvel > 0){
    Kvel -= 0.01;
    preferences.putFloat("Kvel", Kvel);
  }
  handleRoot();
}
void KvelP() {
  if(Kvel <= 30){
    Kvel += 0.01;
    preferences.putFloat("Kvel", Kvel);
  }
  handleRoot();
}

void KrollM() {
  if(Kroll > 0){
    Kroll -= 0.1;
    preferences.putFloat("Kroll", Kroll);
  }
  handleRoot();
}
void KrollP() {
  if(Kroll <= 30){
    Kroll += 0.1;
    preferences.putFloat("Kroll", Kroll);
  }
  handleRoot();
}

void KyawM() {
  if(Kyaw >= -10){
    Kyaw -= 0.01;
    preferences.putFloat("Kyaw", Kyaw);
  }
  handleRoot();
}
void KyawP() {
  if(Kyaw <= 10){
    Kyaw += 0.01;
    preferences.putFloat("Kyaw", Kyaw);
  }
  handleRoot();
}

void DelayM() {
  if(Delay > 0){
    Delay -= 1;
    preferences.putInt("Delay", Delay);
  }
  handleRoot();
}
void DelayP() {
  if(Delay <= 30){
    Delay += 1;
    preferences.putInt("Delay", Delay);
  }
  handleRoot();
}


//Core0 ディスプレイ表示
void display(void *pvParameters) {
  Serial.println("-----------------------------");
  uint8_t btmac[6];
  esp_read_mac(btmac, ESP_MAC_BT);
  Serial.printf("[Bluetooth] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", btmac[0], btmac[1], btmac[2], btmac[3], btmac[4], btmac[5]);

  PS4.begin();

  M5.dis.setBrightness(20);

  Serial1.begin(1000000, SERIAL_8N1, -1, 33); //RX, TX
  delay(100);
  

  WiFi.softAP(ssid, pass);           // SSIDとパスの設定
  delay(100);
  WiFi.softAPConfig(ip, ip, subnet); // IPアドレス、ゲートウェイ、サブネットマスクの設定
  
  IPAddress myIP = WiFi.softAPIP();  // WiFi起動
  

  server.on("/", handleRoot); 

  server.on("/KpP", KpP);
  server.on("/KpM", KpM);
  server.on("/KdP", KdP);
  server.on("/KdM", KdM);
  server.on("/KwP", KwP);
  server.on("/KwM", KwM);
  server.on("/KiP", KiP);
  server.on("/KiM", KiM);

  server.on("/KvelP", KvelP);
  server.on("/KvelM", KvelM);
  server.on("/KrollP", KrollP);
  server.on("/KrollM", KrollM);
  
  server.on("/KyawP", KyawP);
  server.on("/KyawM", KyawM);
  
  server.on("/DelayM", DelayM);
  server.on("/DelayP", DelayP);
  
  server.begin();
  
  for (;;){
    server.handleClient();

    if (PS4.isConnected()) { //PS4コントローラ
      //ジョイスティック
      omegaCmd = -PS4.LStickX() * Kroll;
      velCmd = -PS4.LStickY() * Kvel / 200.0;

      if (PS4.Options()){
        GetUP = 0;
        delay(1000);
      }
    }

    
    //LED表示
    M5.dis.clear();
      
    if(kalAngleX > 20.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i, 0xff0000);
      }
    }else if(kalAngleX <= 20.0 && kalAngleX > 12.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i, 0x0000ff);
      }
    }else if(kalAngleX <= 12.0 && kalAngleX > 4.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 5, 0x0000ff);
      }
    }else if(kalAngleX <= 4.0 && kalAngleX > 1.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 10, 0x0000ff);
      }
    }else if(abs(kalAngleX) <= 1.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 10, 0x00ff00);
      }
    }else if(kalAngleX >= -4.0 && kalAngleX < -1.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 10, 0x0000ff);
      }
    }else if(kalAngleX >= -12.0 && kalAngleX < -4.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 15, 0x0000ff);
      }
    }else if(kalAngleX >= -20.0 && kalAngleX < -12.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 20, 0x0000ff);
      }
    }else if(kalAngleX < -20.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i + 20, 0xff0000);
      }
    }
      
    if(kalAngleY > 20.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i * 5, 0xff0000);
      }
    }else if(kalAngleY <= 20.0 && kalAngleY > 12.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i * 5, 0x0000ff);
      }
    }else if(kalAngleY <= 12.0 && kalAngleY > 4.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i * 5 + 1, 0x0000ff);
      }
    }else if(kalAngleY <= 4.0 && kalAngleY > 1.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i * 5 + 2, 0x0000ff);
      }
    }else if(abs(kalAngleY) <= 1.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i * 5 + 2, 0x00ff00);
      }
    }else if(kalAngleY >= -4.0 && kalAngleY < -1.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i * 5 + 2, 0x0000ff);
      }
    }else if(kalAngleY >= -12.0 && kalAngleY < -4.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i * 5 + 3, 0x0000ff);
      }
    }else if(kalAngleY >= -20.0 && kalAngleY < -12.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i * 5 + 4, 0x0000ff);
      }
    }else if(kalAngleY < -20.0){
      for(int i = 0; i < 5; i++){
        M5.dis.drawpix(i * 5 + 4, 0xff0000);
      }
    }

    scs_setSpeed(servoL, ML1);
    scs_setSpeed(servoR, MR1);

    disableCore0WDT();
  }
}


void setup() {
  M5.begin(true, true, true); //SerialEnable, bool I2CEnable, DisplayEnable
  
  // IMU初期化
  M5.IMU.Init();
  

  //フルスケールレンジ
  M5.IMU.SetAccelFsr(M5.IMU.AFS_2G);
  M5.IMU.SetGyroFsr(M5.IMU.GFS_250DPS);
   
  get_theta();
  kalmanX.setAngle(theta_X);
  kalmanY.setAngle(theta_Y);


  preferences.begin("parameter", false);

  Kp = preferences.getFloat("Kp",Kp);
  Kd = preferences.getFloat("Kd",Kd);
  Kw = preferences.getFloat("Kw",Kw);
  Ki = preferences.getFloat("Ki",Ki);

  Kvel = preferences.getFloat("Kvel",Kvel);
  Kroll = preferences.getFloat("Kroll",Kroll);
  
  Kyaw = preferences.getFloat("Kyaw",Kyaw);
  Delay = preferences.getInt("Delay",Delay);


  //core0 ディスプレイ表示
  xTaskCreatePinnedToCore(
    display
    ,  "display"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);
}


void loop() {
  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  
  dt = (float)loopTime / 1000000.0; //sec

  get_theta();
  get_gyro();

  
  //カルマンフィルタ 姿勢 傾き
  kalAngleX = kalmanX.getAngle(theta_X, theta_Xdot, dt);
  kalAngleY = kalmanY.getAngle(theta_Y, theta_Ydot, dt);
  
  //カルマンフィルタ 姿勢 角速度
  kalAngleDotX = kalmanX.getRate();
  kalAngleDotY = kalmanY.getRate();
  

  //モータ起動
  if (fabs(kalAngleY) < 1 && GetUP == 0){
    GetUP = 1;
  }
    
  if(GetUP == 1){
    //ブレーキ
    if(fabs(kalAngleY) > 40.0){
      ML1 = 0;
      MR1 = 0;
      GetUP = 0;
      theta0_hat = 0.0;
    }


    velCmdFiler += 0.1 * (velCmd - velCmdFiler);
    
        
    //モータ制御
    M = Kp / 10.0 * (kalAngleY + theta0_hat)
      + Kd / 500.0 * kalAngleDotY
      + Kw / 1000.0 * (vel - velCmdFiler);
    
    Tturn = Kyaw / 100.0 * (theta_Zdot - omegaCmd);
    ML1 = M - Tturn;
    MR1 = M + Tturn;

    ML1 = constrain(ML1, -1.0, 1.0);
    MR1 = constrain(MR1, -1.0, 1.0);


    //モータ回転速度
    vel = (ML1 + MR1) * 0.5;
    theta0_hat += Ki * 100.0 * (vel - velCmdFiler) * dt;
    theta0_hat *= 0.99;
    
    ML1 = -1023.0 * ML1;
    MR1 = 1023.0 * MR1;

  }else if(GetUP == 0){ //モータ停止
    ML1 = 0;
    MR1 = 0;
  }
    
  delay(Delay);
}
