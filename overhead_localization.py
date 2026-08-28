"""
============================================================================
Localizacion cenital del robot (webcam + ArUco) -> envio de pose por UDP
============================================================================
- Camara fija mirando hacia abajo (cenital), conectada a la PC.
- Un marcador ArUco pegado arriba del robot da posicion + orientacion.
- Una homografia (calibrada una sola vez, con 4 puntos conocidos del piso)
  convierte pixeles -> centimetros reales.
- La pose (x, y, theta) se manda por UDP al ESP8266, que la reenvia al
  Arduino UNO.
- Un click con el mouse sobre la imagen define el destino y lo envia una
  sola vez como "G,x,y".

Requisitos:
    pip install opencv-contrib-python numpy

Antes de usarlo:
  1) Imprimi un marcador ArUco (diccionario DICT_4X4_50, id 0) y pegalo
     centrado y bien plano arriba del robot, alineado con su eje "adelante".
  2) Definí ROBOT_ID y CAM_INDEX abajo.
  3) Cambiá ESP8266_IP por la IP impresa en el monitor serie del ESP8266.
  4) Corré el script, hacé click en las 4 esquinas del area de trabajo en
     el orden que pide la calibracion (ver funcion calibrar_homografia).
============================================================================
"""

import cv2
import numpy as np
import socket
import time

# ---------------- Configuracion ----------------
CAM_INDEX = 0
ROBOT_ARUCO_ID = 0
ARUCO_DICT = cv2.aruco.DICT_4X4_50

ESP8266_IP = "192.168.1.50"   # <-- reemplazar con la IP real del ESP8266
ESP8266_PORT = 4210
SEND_HZ = 5.0                 # frecuencia de envio de pose (Hz)

# Dimensiones reales del area de trabajo, en cm (ancho x alto),
# usadas para la calibracion de la homografia.
AREA_ANCHO_CM = 200.0
AREA_ALTO_CM = 150.0

# ---------------- UDP ----------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def enviar_udp(mensaje: str):
    sock.sendto(mensaje.encode("utf-8"), (ESP8266_IP, ESP8266_PORT))


# ---------------- Calibracion de homografia (pixeles -> cm) ----------------
puntos_click = []


def click_calibracion(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(puntos_click) < 4:
        puntos_click.append((x, y))
        print(f"Punto {len(puntos_click)}: ({x},{y})")


def calibrar_homografia(cap) -> np.ndarray:
    """
    Pide clickear, en orden, las 4 esquinas del area de trabajo sobre el piso:
    1) esquina superior-izquierda  2) superior-derecha
    3) inferior-derecha            4) inferior-izquierda
    (mirando la imagen de la camara tal como se ve en pantalla)
    Devuelve la matriz de homografia que mapea pixeles -> centimetros reales.
    """
    global puntos_click
    puntos_click = []
    cv2.namedWindow("Calibracion")
    cv2.setMouseCallback("Calibracion", click_calibracion)

    print("Click en las 4 esquinas del area de trabajo (sentido horario, "
          "empezando arriba-izquierda). ESC para cancelar.")

    while len(puntos_click) < 4:
        ok, frame = cap.read()
        if not ok:
            continue
        for i, p in enumerate(puntos_click):
            cv2.circle(frame, p, 6, (0, 255, 0), -1)
        cv2.imshow("Calibracion", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            raise SystemExit("Calibracion cancelada.")

    cv2.destroyWindow("Calibracion")

    pixel_pts = np.array(puntos_click, dtype=np.float32)
    mundo_pts = np.array([
        [0, 0],
        [AREA_ANCHO_CM, 0],
        [AREA_ANCHO_CM, AREA_ALTO_CM],
        [0, AREA_ALTO_CM],
    ], dtype=np.float32)

    H, _ = cv2.findHomography(pixel_pts, mundo_pts)
    print("Homografia calculada.")
    return H


def pixel_a_mundo(H: np.ndarray, punto_px) -> tuple:
    p = np.array([punto_px[0], punto_px[1], 1.0])
    w = H @ p
    w /= w[2]
    return float(w[0]), float(w[1])


# ---------------- Deteccion ArUco y pose ----------------
def detectar_pose_robot(frame, detector, H):
    """
    Devuelve (x_cm, y_cm, theta_rad) del robot, o None si no se detecto.
    theta se mide como el angulo del vector "atras -> adelante" del marcador,
    ya en el sistema de coordenadas del mundo (tras la homografia).
    """
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return None

    for i, marker_id in enumerate(ids.flatten()):
        if marker_id != ROBOT_ARUCO_ID:
            continue

        c = corners[i][0]  # 4 esquinas (px): [sup-izq, sup-der, inf-der, inf-izq]
        centro_px = c.mean(axis=0)

        # Punto "adelante" del marcador: punto medio entre esquina sup-izq y sup-der
        # (asumiendo que el marcador se pego con ese borde apuntando al frente
        # del robot -- ajustar si tu orientacion fisica es distinta)
        frente_px = (c[0] + c[1]) / 2.0

        x_c, y_c = pixel_a_mundo(H, centro_px)
        x_f, y_f = pixel_a_mundo(H, frente_px)

        theta = np.arctan2(y_f - y_c, x_f - x_c)
        return x_c, y_c, theta, centro_px, frente_px

    return None


# ---------------- Programa principal ----------------
def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("No se pudo abrir la camara.")

    H = calibrar_homografia(cap)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    cv2.namedWindow("Localizacion")

    goal_px = {"pt": None}

    def click_destino(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            goal_px["pt"] = (x, y)
            gx, gy = pixel_a_mundo(H, (x, y))
            print(f"Destino: ({gx:.1f}, {gy:.1f}) cm")
            enviar_udp(f"G,{gx:.1f},{gy:.1f}")

    cv2.setMouseCallback("Localizacion", click_destino)

    ultimo_envio = 0.0
    intervalo = 1.0 / SEND_HZ

    print("Click sobre la imagen para fijar el destino. 'q' para salir.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        resultado = detectar_pose_robot(frame, detector, H)

        if resultado is not None:
            x, y, theta, centro_px, frente_px = resultado

            ahora = time.time()
            if ahora - ultimo_envio >= intervalo:
                enviar_udp(f"P,{x:.1f},{y:.1f},{theta:.4f}")
                ultimo_envio = ahora

            # Overlay de depuracion
            cv2.circle(frame, tuple(centro_px.astype(int)), 5, (0, 0, 255), -1)
            cv2.arrowedLine(frame, tuple(centro_px.astype(int)),
                             tuple(frente_px.astype(int)), (0, 255, 0), 2)
            cv2.putText(frame, f"x={x:.0f} y={y:.0f} th={np.degrees(theta):.0f}deg",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            cv2.putText(frame, "Marcador no detectado", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        if goal_px["pt"] is not None:
            cv2.circle(frame, goal_px["pt"], 8, (255, 0, 0), 2)

        cv2.imshow("Localizacion", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
