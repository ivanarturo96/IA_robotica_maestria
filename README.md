# IA_robotica_maestria

Robot móvil diferencial de dos ruedas basado en **Arduino UNO** que evita obstáculos y navega hacia un destino usando **campos potenciales artificiales (APF)**. La pose del robot la calcula una **cámara cenital** con un marcador **ArUco** y se envía por **WiFi (UDP)** a través de un módulo **ESP-01 (ESP8266)**.

Proyecto desarrollado para la maestría (materia de robótica / IA).

---

## Cómo funciona

```
 ┌──────────────┐   pose (x, y, θ) y destino   ┌─────────────────┐   AT / SoftwareSerial   ┌──────────────┐
 │ PC + cámara  │ ───────────────────────────► │ ESP-01 (WiFi)   │ ◄─────────────────────► │ Arduino UNO  │
 │ Python/OpenCV│        UDP, puerto 4210      │ firmware AT     │       9600 baudios      │ + sensores   │
 └──────────────┘                              └─────────────────┘                         └──────┬───────┘
        ▲                                                                                         │ PWM
        │ imagen cenital                                                                          ▼
   Marcador ArUco sobre el robot                                          Motores DC + servo + sensor ultrasónico
```

1. Una cámara fija mirando hacia abajo ve el área de trabajo y el marcador ArUco pegado sobre el robot.
2. El script de Python calcula la posición y la orientación del robot (en cm y radianes) mediante una homografía calibrada con 4 puntos del piso.
3. La pose se envía por UDP a ~5 Hz. Con un clic sobre la imagen se define el destino.
4. El Arduino combina una fuerza **atractiva** hacia la meta con una fuerza **repulsiva** calculada a partir de un barrido con el sensor ultrasónico montado sobre un servo, y traduce el resultado a velocidades de las ruedas.

## Estructura del repositorio

| Ruta | Descripción |
|---|---|
| `campo_potencial/potential_field_navigation_v3.ino` | Sketch principal: navegación por campos potenciales con comunicación WiFi/UDP. |
| `control_python/overhead_localization.ipynb` | Localización cenital (OpenCV + ArUco), calibración de homografía y envío de pose/destino por UDP. |
| `Obstacle_Avoidance_Robot_Car.ino` | Sketch base de evasión de obstáculos reactiva (sin pose ni WiFi). |
| `test_varios/Test_Arduino_ESP_notebook.ino` | Puente serie UNO ↔ ESP-01 para enviar comandos AT desde el monitor serie. |
| `test_varios/min_velocidad_motor_dc.ino` | Rampa de PWM para encontrar el PWM mínimo con el que arranca cada motor. |
| `test_varios/crear_imagen_aruco.ipynb` | Genera la imagen del marcador ArUco (`DICT_4X4_50`, id 23). |
| `test_varios/detectar_camara_iphone.ipynb` | Prueba para usar la cámara del iPhone como webcam (Camo / EpocCam / iVCam). |
| `aruco_marker.pdf` | Marcador ArUco listo para imprimir. |
| `esp01_Sistema_Operativo.png` | Captura de la respuesta `AT+GMR` del ESP-01 (firmware AT 0.40.0.0, SDK 1.3.0). |

## Hardware

- Arduino UNO
- Chasis de robot diferencial de 2 ruedas con 2 motores DC y driver de motores (entradas ENA/ENB e IN1–IN4)
- Sensor ultrasónico (tipo HC-SR04) montado sobre un servo
- Módulo ESP-01 (ESP8266) con firmware AT
- Divisor de voltaje (por ejemplo 1 kΩ / 2 kΩ) o level shifter para la línea TX del Arduino → RX del ESP-01
- Cámara cenital (webcam o celular como webcam) y PC con Python
- Marcador ArUco impreso (ver `aruco_marker.pdf`)

### Pines

| Función | Pin del UNO |
|---|---|
| Ultrasónico ECHO | `A0` |
| Ultrasónico TRIG | `A1` |
| Servo | `A2` |
| Motor izquierdo: PWM (ENA) / IN1 / IN2 | `5` / `2` / `4` |
| Motor derecho: PWM (ENB) / IN3 / IN4 | `6` / `7` / `8` |
| ESP-01 TX → UNO RX (SoftwareSerial) | `10` |
| UNO TX → ESP-01 RX (SoftwareSerial, **con divisor de voltaje**) | `11` |

> ⚠️ El ESP-01 trabaja a 3.3 V y no tolera 5 V en su pin RX.

## Puesta en marcha

### 1. Configurar el ESP-01

`SoftwareSerial` no es confiable a 115200 baudios en un UNO, por lo que hay que dejar el módulo a 9600:

1. Conectá el ESP-01 a un adaptador USB-UART y abrí un monitor serie a 115200.
2. Verificá que responda `OK` a `AT` y revisá la versión con `AT+GMR` (ver `esp01_Sistema_Operativo.png`).
3. Enviá una única vez: `AT+UART_DEF=9600,8,1,0,0` (queda guardado en flash).

Con `test_varios/Test_Arduino_ESP_notebook.ino` podés mandar comandos AT desde el monitor serie del IDE a través del UNO para comprobar el cableado.

### 2. Cargar el firmware del robot

1. En el Arduino IDE instalá la librería **WiFiEsp** (de *bportaluri*) desde *Administrar bibliotecas*.
2. Abrí `campo_potencial/potential_field_navigation_v3.ino` y editá `ssid` y `pass` con los datos de tu red WiFi.
3. Calibrá los PWM mínimos (`SPIN_MIN_*`) con `test_varios/min_velocidad_motor_dc.ino` y medí el robot para ajustar `ROBOT_RADIUS_CM`.
4. Cargá el sketch. Al conectarse, el monitor serie (9600) imprime la **IP del robot**; esa IP se usa en el script de Python.

### 3. Ejecutar la localización cenital

```bash
pip install opencv-contrib-python numpy pygrabber
```

> `pygrabber` usa DirectShow y solo funciona en Windows; se necesita únicamente para seleccionar la cámara del celular por nombre.

1. Imprimí `aruco_marker.pdf` (o generá otro con `test_varios/crear_imagen_aruco.ipynb`) y pegalo plano sobre el robot, con el borde superior apuntando hacia el frente.
2. En `control_python/overhead_localization.ipynb` ajustá `CAM_INDEX`, `ROBOT_ARUCO_ID` (23 por defecto), `ESP8266_IP` y `AREA_ANCHO_CM` / `AREA_ALTO_CM` (150 × 150 cm por defecto).
3. Ejecutá el script y hacé clic en las 4 esquinas del área de trabajo, en sentido horario empezando por arriba a la izquierda.
4. Hacé clic sobre la imagen para fijar el destino. Presioná `q` para salir.

## Protocolo UDP (puerto 4210)

| Mensaje | Significado |
|---|---|
| `P,x,y,theta` | Pose actual del robot: `x`, `y` en cm y `theta` en radianes (sistema de coordenadas del mundo). |
| `G,x,y` | Nuevo destino en cm. |

## Algoritmo de navegación

- **Fuerza atractiva:** proporcional a la distancia a la meta (`K_ATT`), saturada a partir de `D_ATT_THRESH`.
- **Fuerza repulsiva:** el servo barre 5 ángulos (30°, 60°, 90°, 120°, 150°) y, para cada medición menor a `D0`, suma una repulsión proporcional a `K_REP · (1/d − 1/D0) / d²`.
- **Control de orientación:** el robot gira hacia el ángulo de la fuerza resultante con un controlador proporcional (`KP_W`). Si el error de orientación es muy grande, gira en el lugar antes de avanzar.
- **Frenado cerca de la meta:** la velocidad se reduce progresivamente dentro de `BRAKE_DIST`; el destino se considera alcanzado dentro de `GOAL_TOL`.
- **Barrido no bloqueante:** el escaneo es una máquina de estados que avanza un paso por vuelta de `loop()`, de modo que el robot sigue leyendo UDP y actualizando los motores.
- **Recuperación de emergencia:** si el frente está a menos de `CRITICAL_DIST`, el robot frena, retrocede y gira hacia el lado con más despeje (útil ante mínimos locales).
- **Watchdog de pose:** si no llega una pose en `POSE_TIMEOUT_MS` (marcador tapado, WiFi caído), el robot se detiene.

### Parámetros principales (a calibrar)

| Parámetro | Valor actual | Descripción |
|---|---|---|
| `K_ATT` | 1.0 | Ganancia de atracción |
| `K_REP` | 16000 | Ganancia de repulsión |
| `D0` | 40 cm | Distancia de influencia de los obstáculos |
| `D_ATT_THRESH` | 100 cm | Distancia a partir de la cual la atracción se satura |
| `GOAL_TOL` | 8 cm | Tolerancia para dar el destino por alcanzado |
| `BASE_SPEED` / `MAX_PWM` | 70 / 110 | Velocidad base y máxima (PWM) |
| `KP_W` | 40 | Ganancia proporcional de giro |
| `CRITICAL_DIST` | 15 cm | Umbral de la recuperación de emergencia |
| `POSE_TIMEOUT_MS` | 500 ms | Tiempo máximo sin recibir pose |

## Sketch base: evasión reactiva

`Obstacle_Avoidance_Robot_Car.ino` avanza hasta detectar un obstáculo a menos de 20 cm; entonces se detiene, mide las distancias a izquierda y derecha con el servo y gira hacia el lado más despejado. No usa pose ni WiFi y sirve como punto de partida y comparación con el enfoque de campos potenciales.

## Notas

- Las credenciales WiFi y la IP del ESP-01 en los archivos son valores de ejemplo: reemplazalas por las de tu red.
- Los parámetros del campo potencial fueron ajustados de forma experimental; dependen del robot, la superficie y el sensor.
