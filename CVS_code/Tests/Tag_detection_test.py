import cv2
import numpy as np

# Capturar un fotograma de la cámara
cap = cv2.VideoCapture(1)
while True:
  ret, frame = cap.read()
  if not ret:
    break

  # Convertir la imagen a espacio de color HSV
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  # Definir los rangos de los colores en HSV
  lower_yellow = np.array([20, 100, 100])
  upper_yellow = np.array([30, 255, 255])

  lower_colors = {
    'red': np.array([0, 100, 100]),
    'cyan': np.array([85, 100, 100]),
    'green': np.array([60, 100, 100]),
    'magenta': np.array([140, 100, 100])
  }

  upper_colors = {
    'red': np.array([10, 255, 255]),
    'cyan': np.array([105, 255, 255]),
    'green': np.array([80, 255, 255]),
    'magenta': np.array([170, 255, 255])
  }

  # Filtrar colores
  mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

  # Detección de contornos
  contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 500:
      approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
      if len(approx) == 4:
        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)

  # Mostrar el fotograma
  cv2.imshow('Frame', frame)

  # Salir si se presiona la tecla 'q'
  if cv2.waitKey(1) & 0xFF == ord('q'):
      break

cap.release()
cv2.destroyAllWindows()