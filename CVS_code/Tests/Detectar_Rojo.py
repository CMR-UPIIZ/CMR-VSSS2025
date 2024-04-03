import cv2
import numpy as np

lower_colors = {
  'red': np.array([171, 122, 240]),
  'cyan': np.array([85, 100, 100]),
  'green': np.array([60, 100, 100]),
  'magenta': np.array([140, 100, 100]),
  'yellow': np.array([30, 100, 255])
}
upper_colors = {
  'red': np.array([177, 153, 255]),
  'cyan': np.array([105, 255, 255]),
  'green': np.array([80, 255, 255]),
  'magenta': np.array([170, 255, 255]),
  'yellow': np.array([30, 146, 255])
}

def encontrar_cuadrado_aprox(contorno):
  # Aproximar el contorno a un polígono
  approx = cv2.approxPolyDP(contorno, 0.04 * cv2.arcLength(contorno, True), True)

  # Encontrar el rectángulo delimitador del polígono aproximado
  rect = cv2.boundingRect(approx)

  return rect

# Función para encontrar el centro de un rectángulo
def encontrar_centro(rect):
  x, y, w, h = rect
  centro_x = x + (w // 2)
  centro_y = y + (h // 2)
  return (centro_x, centro_y)

# Función para encontrar el ángulo de rotación de un rectángulo
def encontrar_angulo(contorno):
  # Ajustar un rectángulo rotado al contorno
  rect = cv2.minAreaRect(contorno)
  _, _, angulo = rect
  return angulo

# red_detection
def detectar_color_rojo(frame):
  # BGR TO HSV CONVERSION
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  # CREATE A MASK FOR RED
  mask_red = cv2.inRange(hsv, lower_colors['red'], upper_colors['red'])
  mask_yel = cv2.inRange(hsv, lower_colors['yellow'], upper_colors['yellow'])

  # Encontrar contornos en la máscara
  contours1, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  contours2, _ = cv2.findContours(mask_yel, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  contours = contours1 + contours2
  # CONTORNOS
  for contour in contours:
    area = cv2.contourArea(contour)
    if area > 100:  # Filtrar contornos pequeños
      # Calcular el rectángulo aproximado al contorno
      rect = encontrar_cuadrado_aprox(contour)
      # Calcular el centro del rectángulo
      centro = encontrar_centro(rect) 
      # Calcular el ángulo de rotación del contorno
      angulo = encontrar_angulo(contour)
      # Dibujar el cuadrado en la imagen
      box = cv2.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), angulo))
      box = np.int0(box)
      cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
      # Dibujar el centro del cuadrado en la imagen
      cv2.circle(frame, centro, 5, (255, 0, 0), -1)

  return frame

# Función para detectar color rojo
def calibrar_color(frame):
  # Convertir de BGR a HSV
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  # Seleccionar un área en la imagen
  (x, y, w, h) = cv2.selectROI('Seleccionar area', frame, fromCenter=False, showCrosshair=True)
  cv2.destroyWindow('Seleccionar area')
  # Recortar el área seleccionada
  area_seleccionada = hsv[y:y+h, x:x+w]

  # Calcular los vectores de color HSV promedio, más opaco y más brillante
  promedio_color = np.round(np.mean(area_seleccionada, axis=(0, 1))).astype(int)
  color_mas_opaco = np.min(area_seleccionada, axis=(0, 1))
  color_mas_brillante = np.max(area_seleccionada, axis=(0, 1))

  # Imprimir los resultados
  print("Color promedio (H, S, V):", promedio_color)
  print("Color más opaco (H, S, V):", color_mas_opaco)
  print("Color más brillante (H, S, V):", color_mas_brillante)

# Cam capture
cap = cv2.VideoCapture(0)

while True:
  ret, frame = cap.read()

  if not ret:
    break
  frame_con_color_rojo = detectar_color_rojo(frame)

  cv2.imshow("Frame con color rojo detectado", frame_con_color_rojo)

  # Esperar la tecla 's' para iniciar la selección de área
  key = cv2.waitKey(1) & 0xFF
  if key == ord("s"):
    calibrar_color(frame)

  # Salir si se presiona 'q'
  elif key == ord("q"):
    break

cap.release()
cv2.destroyAllWindows()
