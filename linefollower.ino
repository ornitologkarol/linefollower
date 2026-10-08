#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

#define IR0 3
#define IR1 2
#define IR2 1
#define IR3 0
#define IR4 6
#define IR5 5
#define IR6 4
#define ENA 18
#define ENB 23
#define IN1 19
#define IN2 20
#define IN3 21
#define IN4 22

Preferences pref;
int kp=0, kd=0, ki = 0, base_speed = 0, change_speed = 0, counter = 0;
int s0=0, s1=0, s2=0, s3=0, s4=0, s5=0, s6=0;
float error = 0, prev_error = 0, derivative = 0, integral = 0, power = 1;
unsigned long prev_time;
int min_sensor[7];
int max_sensor[7];
bool running = false, left=false;

const char* ssid = "ESP32_Siec_Unikalna";
const char* password = "12345678";
AsyncWebServer server(80);
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pl">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Serwer cos</title>
</head>
<body>
<form action="/get">
  base_speed: <input type="text" name="base_speed">
  <input type="submit" value="Submit">
  <span id="base_speed"></span>
</form>
<br>
<form action="/get">
  change_speed: <input type="text" name="change_speed">
  <input type="submit" value="Submit">
  <span id="change_speed"></span>
</form>
<br>
<form action="/get">
  kp: <input type="text" name="kp">
  <input type="submit" value="Submit">
  <span id="kp"></span>
</form>
<br>
<form action="/get">
  ki: <input type="text" name="ki">
  <input type="submit" value="Submit">
  <span id="ki"></span>
</form>
<br>
<form action="/get">
  kd: <input type="text" name="kd">
  <input type="submit" value="Submit">
  <span id="kd"></span>
</form>
<br>
<form action="/get">
  power: <input type="text" name="power">
  <input type="submit" value="Submit">
  <span id="power"></span>
</form>
<br>
<button onclick="location.href='/mini'">calibracja min</button>
<br><br>
<button onclick="location.href='/maxi'">calibracja max</button>
<br><br>
<button onclick="location.href='/reset'">restart</button>
<br><br>
<button onclick="location.href='/onoff'">on/off</button>
<br><br>
<span id="s0"></span>
<span id="s1"></span>
<span id="s2"></span>
<span id="s3"></span>
<span id="s4"></span>
<span id="s5"></span>
<span id="s6"></span>
</body>
<script>
  function updateVal(id) {
    fetch('/read?param=' + id)
      .then(res => res.text())
      .then(data => document.getElementById(id).innerText = data);
  }

  setInterval(() => {
    ['base_speed', 'change_speed', 'kp', 'ki', 'kd', 'power'].forEach(updateVal);
  }, 1000);
  setInterval(() => {
    ['s0', 's1', 's2', 's3', 's4', 's5', 's6'].forEach(updateVal);
  }, 200);
</script>
</html>
)rawliteral";

int read_avg(int ir_pin) {
  int sum = 0;
  for (int i=0; i < 10; i++) {
    sum += analogRead(ir_pin);
  }
  return sum / 10;
}

void set_motor(int pin_up, int pin_down, int pin_pwm, float speed) {
  if (speed >= 0) {
    digitalWrite(pin_up, HIGH);
    digitalWrite(pin_down, LOW);
  } else {
    digitalWrite(pin_up, LOW);
    digitalWrite(pin_down, HIGH);
  }
  analogWrite(pin_pwm, constrain(abs(speed), 0, 255));
}

void line_follower() {
  s0 = map(constrain(analogRead(IR0), min_sensor[0], max_sensor[0]), min_sensor[0], max_sensor[0], 0, 1000);
  s1 = map(constrain(analogRead(IR1), min_sensor[1], max_sensor[1]), min_sensor[1], max_sensor[1], 0, 1000);
  s2 = map(constrain(analogRead(IR2), min_sensor[2], max_sensor[2]), min_sensor[2], max_sensor[2], 0, 1000);
  s3 = map(constrain(analogRead(IR3), min_sensor[3], max_sensor[3]), min_sensor[3], max_sensor[3], 0, 1000);
  s4 = map(constrain(analogRead(IR4), min_sensor[4], max_sensor[4]), min_sensor[4], max_sensor[4], 0, 1000);
  s5 = map(constrain(analogRead(IR5), min_sensor[5], max_sensor[5]), min_sensor[5], max_sensor[5], 0, 1000);
  s6 = map(constrain(analogRead(IR6), min_sensor[6], max_sensor[6]), min_sensor[6], max_sensor[6], 0, 1000);

  int total = s0 + s1 + s2 + s3 + s4 + s5 + s6;
  float position = 0;
  if (total > 300) {
    position = (s0 * 0.0 + s1 * 1.0 + s2 * 2.0 + s3 * 3.0 + s4 * 4.0 + s5 * 5.0 + s6 * 6.0) / total;
    error = 3.0 - position;
    counter = 0;
    if(error >= 1.0) {
      left=true;
    } else if (error <= -1.0) {
      left = false;
    }
  } else {
    if (counter++ > 50) {
      if (left) {
        error = 3.0;
      } else {
        error = -3.0;
      }
    }
  }

  unsigned long curr_time = millis();
  float dt = (curr_time - prev_time) / 1000;
  if (dt != 0) {
    derivative = (error - prev_error) / dt; 
    integral += error * dt;
  }

  float PID = kp * (error >=0 ? 1 : -1) * pow(abs(error), power) + ki * integral + kd * derivative;
  float speed = base_speed + ( ( 3.0 - fabs(error) ) / 3.0 ) * change_speed;
  Serial.printf("%d %d %d %d %d %d %d %f %d\n", s0,s1,s2,s3,s4,s5,s6,position,counter);
  if (running) {
    set_motor(IN1, IN2, ENA, speed - PID);
    set_motor(IN3, IN4, ENB, speed + PID);
  }
  prev_time = curr_time;
  prev_error = error;

  if (counter >= 800) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    running = false;
    counter = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pref.begin("baza", false); 
  kp = pref.getInt("kp", 0);
  kd = pref.getInt("kd", 0);
  ki = pref.getInt("ki", 0);
  base_speed = pref.getInt("base_speed", 0);
  change_speed = pref.getInt("change_speed", 0);
  power = pref.getFloat("power", 1.0);
  for (int i=0; i < 7; i++) {
    min_sensor[i] = pref.getInt((String("min")+i).c_str(), 0);
    max_sensor[i] = pref.getInt((String("max")+i).c_str(), 4095);
  }
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA, base_speed);
  analogWrite(ENB, base_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  if (MDNS.begin("esp32")) {
    Serial.println("mDNS aktywne: http://esp32.local");
  }
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("base_speed")) {
      base_speed = request->getParam("base_speed")->value().toInt();
      pref.putInt("base_speed", base_speed);
    } else if (request->hasParam("change_speed")) {
      change_speed = request->getParam("change_speed")->value().toInt();
      pref.putInt("change_speed", change_speed);
    } else if (request->hasParam("kp")) {
      kp = request->getParam("kp")->value().toInt();
      pref.putInt("kp", kp);
    } else if (request->hasParam("ki")) {
      ki = request->getParam("ki")->value().toInt();
      pref.putInt("ki", ki);
    } else if (request->hasParam("kd")) {
      kd = request->getParam("kd")->value().toInt();
      pref.putInt("kd", kd);
    } else if (request->hasParam("power")) {
      power = request->getParam("power")->value().toFloat();
      pref.putFloat("power", power);
    }
    request->send_P(200, "text/html", index_html);
  });
  server.on("/mini", HTTP_GET, [](AsyncWebServerRequest *request){
    min_sensor[0] = read_avg(IR0);
    min_sensor[1] = read_avg(IR1);
    min_sensor[2] = read_avg(IR2);
    min_sensor[3] = read_avg(IR3);
    min_sensor[4] = read_avg(IR4);
    min_sensor[5] = read_avg(IR5);
    min_sensor[6] = read_avg(IR6);
    for (int i=0; i < 7; i++) {
      pref.putInt((String("min")+i).c_str(), min_sensor[i]);
    }
    Serial.println("Kalibracja min");
    request->send_P(200, "text/html", index_html);
  });
  server.on("/maxi", HTTP_GET, [](AsyncWebServerRequest *request){
    max_sensor[0] = read_avg(IR0);
    max_sensor[1] = read_avg(IR1);
    max_sensor[2] = read_avg(IR2);
    max_sensor[3] = read_avg(IR3);
    max_sensor[4] = read_avg(IR4);
    max_sensor[5] = read_avg(IR5);
    max_sensor[6] = read_avg(IR6);
    for (int i=0; i < 7; i++) {
      pref.putInt((String("max")+i).c_str(), max_sensor[i]);
    }
    Serial.println("Kalibracja max");
    request->send_P(200, "text/html", index_html);
  });
  server.on("/onoff", HTTP_GET, [](AsyncWebServerRequest *request){
    running = !running;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("on/off");
    request->send_P(200, "text/html", index_html);
  });
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    for(int i=0; i < 7; i++) {
      min_sensor[i] = 0;
      max_sensor[i] = 4095;
      pref.putInt((String("max")+i).c_str(), max_sensor[i]);
      pref.putInt((String("min")+i).c_str(), min_sensor[i]);
    }
    request->send_P(200, "text/html", index_html);
  });
  server.on("/read", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("param")) {
      String param =request->getParam("param")->value();
      if (param == "base_speed") {
        request->send(200, "text/plain", String(base_speed));
      } else if (param == "change_speed") {
        request->send(200, "text/plain", String(change_speed));
      } else if (param == "kp") {
        request->send(200, "text/plain", String(kp));
      } else if (param == "kd") {
        request->send(200, "text/plain", String(kd));
      } else if (param == "ki") {
        request->send(200, "text/plain", String(ki));
      } else if (param == "power") {
        request->send(200, "text/plain", String(power));
      } else if (param == "power") {
        request->send(200, "text/plain", String(power));
      } else if (param == "s0") {
        request->send(200, "text/plain", String(s0));
      } else if (param == "s1") {
        request->send(200, "text/plain", String(s1));
      } else if (param == "s2") {
        request->send(200, "text/plain", String(s2));
      } else if (param == "s3") {
        request->send(200, "text/plain", String(s3));
      } else if (param == "s4") {
        request->send(200, "text/plain", String(s4));
      } else if (param == "s5") {
        request->send(200, "text/plain", String(s5));
      } else if (param == "s6") {
        request->send(200, "text/plain", String(s6));
      } else {
        request->send(200, "text/plain", "None");
      }
    }
  });
  server.begin();

  prev_time = millis();
}

void loop() {
  line_follower();
}
