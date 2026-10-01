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
const int pinLB = 2;      
const int pinLF = 4;
const int pinRB = 7;      
const int pinRF = 8;

// PWM minimo con el que cada motor realmente arranca, con el robot en el suelo.
// Calibrar (ver abajo). Valores de ejemplo:
const int SPIN_MIN_L_FWD = 60;
const int SPIN_MIN_L_REV = 60;
const int SPIN_MIN_R_FWD = 60;
const int SPIN_MIN_R_REV = 60;   // el sentido que te cuesta suele necesitar mas

// SoftwareSerial hacia el ESP-01 (firmware AT, ver notas de baudrate arriba)
const int ESP_RX_PIN = 10;  // Arduino RX  <- ESP-01 TX
const int ESP_TX_PIN = 11;  // Arduino TX  -> ESP-01 RX (con divisor de voltaje!)
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

// ---------------- WiFi / UDP ----------------
char ssid[] = "LAN_to_use"; 
char pass[] = "password"; 
int wifiStatus = WL_IDLE_STATUS;
WiFiEspUDP udp;
const unsigned int LOCAL_UDP_PORT = 4210;
char packetBuffer[64];

// ---------------- Parametros del campo potencial (a calibrar) ----------------
const float K_ATT = 1.0;
const float K_REP = 16000.0; //EDITO 8000 -> 16000
const float D0 = 40.0; 
const float D_ATT_THRESH = 100.0; //EDITO 60 -> 100
const float GOAL_TOL = 8.0;

const int BASE_SPEED = 70; // EDITO 140 -> 60
const int MAX_PWM = 110; // EDITO 220 -> 110 (tal vez 90)
const float KP_W = 40.0; // EDITO 90 -> 50 (tal vez 50)
const float HEADING_STOP_ANGLE = 1.05;

const int NUM_ANGLES = 5;
const int SCAN_ANGLES[NUM_ANGLES] = {150, 120, 90, 60, 30};
const int SETTLE_MS = 500; //EDITO 150 -> 500

const float BRAKE_DIST = 10.0; // AGREGO EDITAR: radio donde empieza a frenar

// ---------------- Dimension del robot (para "inflar" obstaculos) ----------------
// El campo potencial trata al robot como un punto. Si dos obstaculos dejan un
// hueco mas angosto que el ANCHO REAL del chasis, el "punto" puede pasar bien
// matematicamente y el cuerpo choca igual. ROBOT_RADIUS_CM = mitad del ancho
// del chasis + margen de seguridad (ej. robot de 16cm de ancho -> 8cm de radio
// + 3cm de margen = 11cm). MEDI TU ROBOT Y AJUSTA ESTE VALOR.
const float ROBOT_RADIUS_CM = 2.0; 

// ---------------- Recuperacion de emergencia (frenar + retroceder + girar) ----------------
// Red de seguridad para cuando, a pesar de la inflacion de arriba, el robot
// termina de todas formas muy cerca de un obstaculo de frente (por ejemplo
// porque el barrido con un solo sensor en servo tarda ~3.5s en completarse y
// el campo repulsivo que se usa para navegar puede estar desactualizado). En
// ese caso NO conviene confiar en el campo potencial (puede estar justo en un
// minimo local, como el caso de los 3 obstaculos en linea con la meta):
// mejor frenar, retroceder un poco y girar hacia el lado con mas despeje.
enum RecoveryState { REC_NONE, REC_BACKUP, REC_TURN };
RecoveryState recoveryState = REC_NONE;
unsigned long recoveryStart = 0;
int recoveryTurnDir = 1; // +1 = gira hacia un lado, -1 = hacia el otro (ajustar signo probando)

const float CRITICAL_DIST = 15.0;      // cm: frente mas cerca que esto -> emergencia
const int REVERSE_SPEED = 70;
const unsigned long REVERSE_MS = 500;  // cuanto tiempo retrocede
const unsigned long TURN_MS = 400;     // cuanto tiempo gira en el lugar despues
const int TURN_SPEED = 80;

float lastFrontDist = 999;  // ultima lectura del sensor mirando derecho al frente (angulo 90)

// ---------------- Estado ----------------
float poseX = 0, poseY = 0, poseTheta = 0;
float goalX = 0, goalY = 0;
bool haveGoal = false;
bool goalReached = false;

// ---------------- Watchdog de pose (deteccion de oclusion / perdida de datos) ----------------
unsigned long lastPoseMillis = 0;           // momento del ultimo paquete "P" recibido
const unsigned long POSE_TIMEOUT_MS = 500;  // si no llega pose en este tiempo, se para el robot

// ---------------- Escaneo repulsivo NO bloqueante (maquina de estados) ----------------
// Antes, computeRepulsiveRobotFrame() bloqueaba el loop() por ~3.5 segundos en cada
// vuelta (5 angulos x (SETTLE_MS + medicion + delay extra) + delay final). Durante ese
// tiempo no se leia UDP ni se recalculaban los motores, por lo que el robot avanzaba
// "a ciegas" con el comando viejo y luego corregia de golpe al recibir pose actualizada
// -> eso es lo que causaba el giro brusco sobre el eje. Ahora el barrido avanza de a un
// paso por vuelta de loop(), sin bloquear.
enum ScanState { SCAN_MOVE, SCAN_SETTLE, SCAN_MEASURE, SCAN_CENTER, SCAN_WAIT_CENTER };
ScanState scanState = SCAN_MOVE;
int scanIndex = 0;
unsigned long scanStateStart = 0;
const unsigned long SCAN_CENTER_MS = 200; // tiempo para que el servo vuelva a 90 grados
float FrxAccum = 0, FryAccum = 0;         // acumulador del barrido en curso
float Frx_cached = 0, Fry_cached = 0;     // ultimo campo repulsivo COMPLETO (frame robot)

// ============================================================================
float checkdistance() {
  digitalWrite(Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig_Pin, LOW);
  float distance = pulseIn(Echo_Pin, HIGH, 25000) / 58.00;
  //if (distance <= 0) distance = 400; COMENTO
  delay(10); //AGREGO
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
      lastPoseMillis = millis(); // marca que llego pose fresca (para el watchdog)
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
  // OJO: antes se leia un solo paquete por vuelta de loop(). Si por cualquier motivo
  // (bloqueos previos, jitter de WiFi) se acumulan varios paquetes en el buffer, hay
  // que procesarlos TODOS para quedarnos con el mas reciente, no con uno viejo en cola.
  int packetSize;
  while ((packetSize = udp.parsePacket()) > 0) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) {
      packetBuffer[len] = 0;
      parsePacketBuffer(packetBuffer);
    }
  }
}

float normalizeAngle(float a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

// Avanza UN paso del barrido por cada llamada (se llama una vez por vuelta de loop()).
// Nunca usa delay() largo: solo compara millis() contra el tiempo del paso anterior.
void updateScanStateMachine() {
  unsigned long now = millis();
  switch (scanState) {
    case SCAN_MOVE:
      myservo.write(SCAN_ANGLES[scanIndex]);
      scanStateStart = now;
      scanState = SCAN_SETTLE;
      break;

    case SCAN_SETTLE:
      if (now - scanStateStart >= SETTLE_MS) {
        scanState = SCAN_MEASURE;
      }
      break;

    case SCAN_MEASURE: {
      float d = checkdistance(); // el pulseIn interno (<=25ms) es aceptable en un solo paso
      //if (d <= 0) d = 400; // sin eco -> se interpreta como "vía libre"

      // Guardamos la lectura de frente (angulo 90 = indice 2 en SCAN_ANGLES)
      // para el chequeo de emergencia de runRecovery(), independiente de si
      // entra o no en el rango D0 usado por el campo potencial.
      if (scanIndex == 2) lastFrontDist = d;

      // Inflamos el obstaculo por el radio del robot: calculamos el campo
      // como si estuviera mas cerca de lo que mide el sensor (dEff < d), y
      // extendemos el rango de deteccion en esa misma medida. Asi, aunque el
      // sensor solo "ve" el punto central del obstaculo, la repulsion empieza
      // a actuar antes y con mas fuerza, dejando margen para el cuerpo real
      // del robot (no solo para su centro).
      if (d > 0 && d < D0) {
        //float deff = d - ROBOT_RADIUS_CM;
        if (d < 5.0) d = 5.0; // AGREGO
        float mag = K_REP * (1.0 / d - 1.0 / D0) / (d * d);
        float alpha = radians((float)(SCAN_ANGLES[scanIndex] - 90));
        FrxAccum += mag * cos(alpha); // EDITO FrxAccum += -mag * cos(alpha); -> FrxAccum += mag * cos(alpha);
        FryAccum += mag * sin(alpha); // EDITO FryAccum += -mag * sin(alpha); -> FryAccum += mag * sin(alpha); 
      }
      scanIndex++;
      scanState = (scanIndex >= NUM_ANGLES) ? SCAN_CENTER : SCAN_MOVE;
      break;
    }

    case SCAN_CENTER:
      myservo.write(90);
      scanStateStart = now;
      scanState = SCAN_WAIT_CENTER;
      break;

    case SCAN_WAIT_CENTER:
      if (now - scanStateStart >= SCAN_CENTER_MS) {
        // Barrido completo: publicar el resultado y arrancar uno nuevo
        Frx_cached = FrxAccum;
        Fry_cached = FryAccum;
        FrxAccum = 0; FryAccum = 0;
        scanIndex = 0;
        scanState = SCAN_MOVE;
      }
      break;
  }
}

// Ejecuta un paso de la recuperacion de emergencia (no bloqueante, igual que
// el barrido). Se llama en loop() mientras recoveryState != REC_NONE.
void runRecovery() {
  unsigned long now = millis();
  if (recoveryState == REC_BACKUP) {
    setLeftMotor(-REVERSE_SPEED);
    setRightMotor(-REVERSE_SPEED);
    if (now - recoveryStart >= REVERSE_MS) {
      recoveryState = REC_TURN;
      recoveryStart = now;
    }
  } else if (recoveryState == REC_TURN) {
    setLeftMotor(recoveryTurnDir * TURN_SPEED);
    setRightMotor(-recoveryTurnDir * TURN_SPEED);
    if (now - recoveryStart >= TURN_MS) {
      recoveryState = REC_NONE;
      lastFrontDist = 999; // forzar a esperar una lectura fresca antes de re-evaluar
      Serial.println("Recuperacion terminada, retomando navegacion normal.");
    }
  }
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
  updateScanStateMachine(); // avanza el barrido del sensor un paso, sin bloquear

  // Prioridad maxima: si ya estamos en medio de una recuperacion (retroceso o
  // giro), terminarla antes de volver a confiar en el campo potencial.
  if (recoveryState != REC_NONE) {
    runRecovery();
    return;
  }

  // Si el frente esta demasiado cerca, NO confiamos en el campo potencial en
  // este instante (puede estar en un minimo local, como el caso de 3
  // obstaculos en linea con la meta): frenamos y arrancamos la recuperacion
  // en vez de seguir calculando velocidades con la formula de APF.
  if (lastFrontDist < CRITICAL_DIST) {
    Serial.print("EMERGENCIA: frente a "); Serial.print(lastFrontDist);
    Serial.println(" cm. Retrocediendo y girando.");
    stopMotors();
    recoveryState = REC_BACKUP;
    recoveryStart = millis();
    // Giramos hacia el lado con mas despeje segun el ultimo barrido: si
    // Fry_cached empuja hacia un lado, giramos hacia ese mismo lado (ajustar
    // el signo probando en la practica, depende de la convencion del robot).
    recoveryTurnDir = (Fry_cached >= 0) ? 1 : -1;
    return;
  }

  // Watchdog: si hace mas de POSE_TIMEOUT_MS que no llega un paquete de pose valido
  // (marcador ArUco tapado, WiFi caido, etc.), no confiar en poseX/poseY/poseTheta
  // viejos: frenar en vez de navegar a ciegas.
  bool poseStale = (millis() - lastPoseMillis > POSE_TIMEOUT_MS);
  if (poseStale) {
    stopMotors();
    return;
  }

  if (!haveGoal || goalReached) {
    stopMotors();
    return;
  }

  float dx = goalX - poseX;
  float dy = goalY - poseY;
  float dist = sqrt(dx * dx + dy * dy);

  float brakeFactor = constrain(dist / BRAKE_DIST, 0.4, 1.0); //AGREGO 0.35 = velocidad mínima para no trabarse por fricción (0.55 con base speed en 30)

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

  // Campo repulsivo: se usa el ultimo barrido COMPLETO ya calculado (Frx_cached /
  // Fry_cached), actualizado en segundo plano por updateScanStateMachine(). Esto
  // permite que el loop corra rapido (lee UDP y recalcula motores en cada vuelta)
  // en vez de congelarse ~3.5s esperando el sweep del servo.
  float Frx_r = Frx_cached, Fry_r = Fry_cached;
  float ct = cos(poseTheta), st = sin(poseTheta); 
  float Frepx = Frx_r * ct - Fry_r * st;
  float Frepy = Frx_r * st + Fry_r * ct;

  float Fx = Fattx + Frepx;
  float Fy = Fatty + Frepy;
  float thetaDes = atan2(Fy, Fx);
  float headingError = normalizeAngle(thetaDes - poseTheta); 

  float w = KP_W * headingError; 

  //bool spinning = false; // AGREGO
  int v;
  if (abs(headingError) > HEADING_STOP_ANGLE) {
    v = 0;
    //spinning = true; //AGREGO
  } else {
    v = (int)(BASE_SPEED * cos(headingError) * brakeFactor); // EDITO (BASE_SPEED * cos(headingError)) -> (BASE_SPEED * cos(headingError) * brakeFactor)
  }

  int leftSpeed = constrain((int)(-w + v), -MAX_PWM, MAX_PWM); // v - w 
  int rightSpeed = constrain((int)(w + v), -MAX_PWM, MAX_PWM); // v + w (-w-v para posetheta horario)

  /*if (spinning) {
    leftSpeed  = (leftSpeed  >= 0) ? max(leftSpeed,  SPIN_MIN_L_FWD) : min(leftSpeed,  -SPIN_MIN_L_REV);
    rightSpeed = (rightSpeed >= 0) ? max(rightSpeed, SPIN_MIN_R_FWD) : min(rightSpeed, -SPIN_MIN_R_REV);
  }*/ //AGREGO ESTAS LINEAS

  leftSpeed  = (leftSpeed  >= 0) ? max(leftSpeed,  SPIN_MIN_L_FWD) : min(leftSpeed,  -SPIN_MIN_L_REV);
  rightSpeed = (rightSpeed >= 0) ? max(rightSpeed, SPIN_MIN_R_FWD) : min(rightSpeed, -SPIN_MIN_R_REV); // AGREGO ESTAS LÍNEAS

  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);

  Serial.print("pose=("); Serial.print(poseX); Serial.print(",");
  Serial.print(poseY); Serial.print(","); Serial.print(poseTheta);
  Serial.print(") err="); Serial.print(headingError);
  Serial.print(" L="); Serial.print(leftSpeed);
  Serial.print(" R="); Serial.println(rightSpeed);
  
}
