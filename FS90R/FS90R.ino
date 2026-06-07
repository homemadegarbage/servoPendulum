#include <M5Atom.h>
#include <Kalman.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PS4Controller.h>
#include <M5AtomicMotion.h>

#define servoL 0
#define servoR 2

M5AtomicMotion AtomicMotion;

WebServer server(80);
const char ssid[] = "AtomMotionServo";  // SSID (任意)
const char pass[] = "password";   // password (任意)
const IPAddress ip(192, 168, 55, 90);      //　IP address (任意)
const IPAddress subnet(255, 255, 255, 0);


int offsetL = 0, offsetR = 0;

unsigned long oldTime = 0, loopTime, nowTime;
float dt;


float Kp = 0.8, Kd = 1.0 ,Kw = 1.0;
float Ki = 2.5;
float Kvel = 0.2;
float Kroll = 1.7;
float Kyaw = 0.1; 
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
  theta_X =  atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  theta_Y  = -atan2(-accX, -accZ) * RAD_TO_DEG;
}

//角速度取得
void get_gyro() {
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  theta_Xdot = -gyroX;
  theta_Ydot = gyroY;
  theta_Zdot = -gyroZ;
}


//ブラウザ表示
void handleRoot() {
  String temp ="<!DOCTYPE html> \n<html lang=\"ja\">";
  temp +="<head>";
  temp +="<meta charset=\"utf-8\">";
  temp +="<title>AtomMotionServo</title>";
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
  

  //offsetL
  temp +="offsetL<br>";
  temp +="<a class=\"pm\" href=\"/offsetLM\">-</a>";
  temp +="<span>" + String(offsetL) + "</span>";
  temp +="<a class=\"pm\" href=\"/offsetLP\">+</a><br>";

  //offsetR
  temp +="offsetR<br>";
  temp +="<a class=\"pm\" href=\"/offsetRM\">-</a>";
  temp +="<span>" + String(offsetR) + "</span>";
  temp +="<a class=\"pm\" href=\"/offsetRP\">+</a><br>";
  
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


void offsetLM() {
  if(offsetL >= -1000){
    offsetL -= 1;
    preferences.putInt("offsetL", offsetL);
  }
  handleRoot();
}
void offsetLP() {
  if(offsetL <= 1000){
    offsetL += 1;
    preferences.putInt("offsetL", offsetL);
  }
  handleRoot();
}

void offsetRM() {
  if(offsetR >= -1000){
    offsetR -= 1;
    preferences.putInt("offsetR", offsetR);
  }
  handleRoot();
}
void offsetRP() {
  if(offsetR <= 1000){
    offsetR += 1;
    preferences.putInt("offsetR", offsetR);
  }
  handleRoot();
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
  

  WiFi.softAP(ssid, pass);           // SSIDとパスの設定
  delay(100);
  WiFi.softAPConfig(ip, ip, subnet); // IPアドレス、ゲートウェイ、サブネットマスクの設定
  
  IPAddress myIP = WiFi.softAPIP();  // WiFi起動
  

  server.on("/", handleRoot); 

  server.on("/offsetLP", offsetLP);
  server.on("/offsetLM", offsetLM);
  server.on("/offsetRP", offsetRP);
  server.on("/offsetRM", offsetRM);

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

    disableCore0WDT();
  }
}


void setup() {
  M5.begin(true, true, true); //SerialEnable, bool I2CEnable, DisplayEnable
  
  // IMU初期化
  M5.IMU.Init();
  AtomicMotion.begin(&Wire1, M5_ATOMIC_MOTION_I2C_ADDR);
  Wire1.setClock(400000);
  

  //フルスケールレンジ
  M5.IMU.SetAccelFsr(M5.IMU.AFS_2G);
  M5.IMU.SetGyroFsr(M5.IMU.GFS_250DPS);
   
  get_theta();
  kalmanX.setAngle(theta_X);
  kalmanY.setAngle(theta_Y);


  preferences.begin("parameter", false);

  offsetL = preferences.getInt("offsetL",offsetL);
  offsetR = preferences.getInt("offsetR",offsetR);

  Kp = preferences.getFloat("Kp",Kp);
  Kd = preferences.getFloat("Kd",Kd);
  Kw = preferences.getFloat("Kw",Kw);
  Ki = preferences.getFloat("Ki",Ki);

  Kvel = preferences.getFloat("Kvel",Kvel);
  Kroll = preferences.getFloat("Kroll",Kroll);
  
  Kyaw = preferences.getFloat("Kyaw",Kyaw);
  Delay = preferences.getInt("Delay",Delay);

  AtomicMotion.setServoPulse(servoL, 1500 + offsetL);
  AtomicMotion.setServoPulse(servoR, 1500 + offsetR);

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
      AtomicMotion.setServoPulse(servoL, 1500 + offsetL);
      AtomicMotion.setServoPulse(servoR, 1500 + offsetR);
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
    
    
    ML1 = -400.0 * ML1;
    MR1 = 400.0 * MR1;

    AtomicMotion.setServoPulse(servoL, 1500 + offsetL + int(ML1));
    AtomicMotion.setServoPulse(servoR, 1500 + offsetR + int(MR1));
  }else if(GetUP == 0){ //モータ停止
    AtomicMotion.setServoPulse(servoL, 1500 + offsetL);
    AtomicMotion.setServoPulse(servoR, 1500 + offsetR);
  }
    
  delay(Delay);
}
