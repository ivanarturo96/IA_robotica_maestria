/*
  ============================================================================
  Navegacion por Campos Potenciales (APF) - Robot diferencial de 2 ruedas
  ESP-01 (ESP8266) con firmware AT, controlado desde el UNO via WiFiEsp
  ============================================================================
  El UNO maneja directamente al ESP-01 (que sigue con su firmware AT de
  fabrica) usando la libreria WiFiEsp, que traduce llamadas tipo WiFi.h en
  comandos AT por SoftwareSerial. El UNO se conecta a la red WiFi, abre un
  socket UDP y escucha ahi los mensajes de pose/destino que manda el script
  de Python. Ya NO hace falta el esp8266_bridge.ino ni reflashear el modulo.

  Instalar antes: Arduino IDE -> Administrar Librerias -> buscar "WiFiEsp"
  (de bportaluri) -> Instalar.

  IMPORTANTE - baudrate del ESP-01:
  SoftwareSerial en un UNO no es confiable a 115200 (baudrate por defecto
  de muchos firmwares AT). Antes de cablear el modulo al UNO, conectalo
  directo al adaptador USB-UART y, con un monitor serie a 115200, mandale
  UNA VEZ:
      AT+UART_DEF=9600,8,1,0,0
  Esto deja el baudrate en 9600 guardado en flash (persiste tras reiniciar).
  Confirma antes con "AT" (debe responder "OK") y "AT+GMR" para ver la
  version de firmware.

  IMPORTANTE - niveles de voltaje:
  El ESP-01 es de 3.3V y NO tolera 5V en su pin RX. El TX del Arduino (5V)
  hacia el RX del ESP-01 necesita un divisor de voltaje (ej. 1k / 2k ohm)
  o un level shifter. El TX del ESP-01 (3.3V) al RX del Arduino se puede
  conectar directo, un UNO lee 3.3V como HIGH sin problema.

  Protocolo UDP esperado (mismo formato que antes, ahora viaja como
  paquetes UDP en vez de lineas de Serial):
    "P,x,y,theta"   -> Pose actual del robot (theta en radianes, world frame)
    "G,x,y"         -> Nuevo destino
  ============================================================================
*/

#include <Servo.h>
#include <SoftwareSerial.h>
#include "WiFiEsp.h"
#include "WiFiEspUdp.h"

Servo myservo;

// ---------------- Pines (igual que el sketch original) ----------------
const int Echo_Pin = A0;
const int Trig_Pin = A1;
const int SERVO_PIN = A2;
const int Lpwm_pin = 5;   // PWM rueda izquierda (ENA)
const int Rpwm_pin = 6;   // PWM rueda derecha (ENB)
const int pinLB = 2;      // direccion rueda izquierda
const int pinLF = 4;
const int pinRB = 7;      // direccion rueda derecha
const int pinRF = 8;

// SoftwareSerial hacia el ESP-01 (firmware AT, ver notas de baudrate arriba)
const int ESP_RX_PIN = 10;  // Arduino RX  <- ESP-01 TX
const int ESP_TX_PIN = 11;  // Arduino TX  -> ESP-01 RX (con divisor de voltaje!)
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

// ---------------- WiFi / UDP ----------------
char ssid[] = "TU_RED_WIFI";
char pass[] = "TU_PASSWORD";
int wifiStatus = WL_IDLE_STATUS;
WiFiEspUDP udp;
const unsigned int LOCAL_UDP_PORT = 4210;
char packetBuffer[64];

// ---------------- Parametros del campo potencial (a calibrar) ----------------
const float K_ATT = 1.0;
const float K_REP = 8000.0;
const float D0 = 40.0;
const float D_ATT_THRESH = 60.0;
const float GOAL_TOL = 8.0;

const int BASE_SPEED = 140;
const int MAX_PWM = 220;
const float KP_W = 90.0;
const float HEADING_STOP_ANGLE = 1.05;

const int NUM_ANGLES = 5;
const int SCAN_ANGLES[NUM_ANGLES] = {150, 120, 90, 60, 30};
const int SETTLE_MS = 150;

// ---------------- Estado ----------------
float poseX = 0, poseY = 0, poseTheta = 0;
float goalX = 0, goalY = 0;
bool haveGoal = false;
bool goalReached = false;

// ============================================================================
float checkdistance() {
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);
  float distance = pulseIn(Echo_Pin, HIGH, 25000) / 58.00;
  if (distance <= 0) distance = 400;
  return distance;
}

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { digitalWrite(pinLB, HIGH); digitalWrite(pinLF, LOW); }
  else            { digitalWrite(pinLB, LOW);  digitalWrite(pinLF, HIGH); }
  analogWrite(Lpwm_pin, abs(speed));
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { digitalWrite(pinRB, LOW);  digitalWrite(pinRF, HIGH); }
  else            { digitalWrite(pinRB, HIGH); digitalWrite(pinRF, LOW); }
  analogWrite(Rpwm_pin, abs(speed));
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

// ---------------- Parseo de paquetes UDP (char*, sin String -> menos RAM) ----------------
void parsePacketBuffer(char* buf) {
  if (buf[0] == 'P') {
    char* tok = strtok(buf + 2, ",");
    if (tok) {
      poseX = atof(tok);
      tok = strtok(NULL, ","); if (tok) poseY = atof(tok);
      tok = strtok(NULL, ","); if (tok) poseTheta = atof(tok);
    }
  } else if (buf[0] == 'G') {
    char* tok = strtok(buf + 2, ",");
    if (tok) {
      goalX = atof(tok);
      tok = strtok(NULL, ",");
      if (tok) {
        goalY = atof(tok);
        haveGoal = true;
        goalReached = false;
        Serial.print("Nuevo destino: ");
        Serial.print(goalX); Serial.print(", "); Serial.println(goalY);
      }
    }
  }
}

void readUdp() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) packetBuffer[len] = 0;
    parsePacketBuffer(packetBuffer);
  }
}

float normalizeAngle(float a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

void computeRepulsiveRobotFrame(float &Frx, float &Fry) {
  Frx = 0; Fry = 0;
  for (int i = 0; i < NUM_ANGLES; i++) {
    int servoAngle = SCAN_ANGLES[i];
    myservo.write(servoAngle);
    delay(SETTLE_MS);
    float d = checkdistance();

    if (d > 0 && d < D0) {
      float mag = K_REP * (1.0 / d - 1.0 / D0) / (d * d);
      float alpha = radians((float)(servoAngle - 90));
      Frx += -mag * cos(alpha);
      Fry += -mag * sin(alpha);
    }
  }
  myservo.write(90);
}

// ============================================================================
void setup() {
  Serial.begin(9600);
  espSerial.begin(9600); // el ESP-01 debe estar configurado a 9600 (ver notas arriba)

  myservo.attach(SERVO_PIN);
  myservo.write(90);

  pinMode(Echo_Pin, INPUT);
  pinMode(Trig_Pin, OUTPUT);
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, OUTPUT);
  pinMode(Rpwm_pin, OUTPUT);
  stopMotors();

  Serial.println("Inicializando ESP-01 (firmware AT)...");
  WiFi.init(&espSerial);

  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("No se detecta el ESP-01. Revisa cableado y baudrate (debe ser 9600).");
    while (true) { delay(1000); }
  }

  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Conectando a: ");
    Serial.println(ssid);
    wifiStatus = WiFi.begin(ssid, pass);
  }
  Serial.println("WiFi conectado.");
  IPAddress ip = WiFi.localIP();
  Serial.print("IP del robot (usar como ESP8266_IP en overhead_localization.py): ");
  Serial.println(ip);

  udp.begin(LOCAL_UDP_PORT);
  Serial.print("Escuchando UDP en puerto ");
  Serial.println(LOCAL_UDP_PORT);
}

void loop() {
  readUdp();

  if (!haveGoal || goalReached) {
    stopMotors();
    return;
  }

  float dx = goalX - poseX;
  float dy = goalY - poseY;
  float dist = sqrt(dx * dx + dy * dy);

  if (dist < GOAL_TOL) {
    stopMotors();
    goalReached = true;
    Serial.println("Destino alcanzado.");
    return;
  }

  float Fattx, Fatty;
  if (dist < D_ATT_THRESH) {
    Fattx = K_ATT * dx;
    Fatty = K_ATT * dy;
  } else {
    Fattx = K_ATT * D_ATT_THRESH * dx / dist;
    Fatty = K_ATT * D_ATT_THRESH * dy / dist;
  }

  float Frx_r, Fry_r;
  computeRepulsiveRobotFrame(Frx_r, Fry_r);
  float ct = cos(poseTheta), st = sin(poseTheta);
  float Frepx = Frx_r * ct - Fry_r * st;
  float Frepy = Frx_r * st + Fry_r * ct;

  float Fx = Fattx + Frepx;
  float Fy = Fatty + Frepy;
  float thetaDes = atan2(Fy, Fx);
  float headingError = normalizeAngle(thetaDes - poseTheta);

  float w = KP_W * headingError;
  int v;
  if (abs(headingError) > HEADING_STOP_ANGLE) {
    v = 0;
  } else {
    v = (int)(BASE_SPEED * cos(headingError));
  }

  int leftSpeed = constrain((int)(v - w), -MAX_PWM, MAX_PWM);
  int rightSpeed = constrain((int)(v + w), -MAX_PWM, MAX_PWM);

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);

  Serial.print("pose=("); Serial.print(poseX); Serial.print(",");
  Serial.print(poseY); Serial.print(","); Serial.print(poseTheta);
  Serial.print(") err="); Serial.print(headingError);
  Serial.print(" L="); Serial.print(leftSpeed);
  Serial.print(" R="); Serial.println(rightSpeed);
}
