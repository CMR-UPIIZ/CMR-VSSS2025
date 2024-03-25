import cv2
import numpy as np


# red_detection
def detectar_color_rojo(frame):
  # BGR TO HSV CONVERSION
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  # RED COLOR RANGES IN HSV
  lower_red = np.array([0, 100, 100])
  upper_red = np.array([10, 255, 255])

  # CREATE A MASK FOR RED
  mask = cv2.inRange(hsv, lower_red, upper_red)

  # Encontrar contornos en la máscara
  contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  # CONTORNOS
  for contour in contours:
    area = cv2.contourArea(contour)
    if area > 100:  # Filtrar contornos pequeños
      cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

  return frame


# Cam capture
cap = cv2.VideoCapture(0)

while True:
  ret, frame = cap.read()

  if not ret:
    break

  frame_con_color_rojo = detectar_color_rojo(frame)

  cv2.imshow("Frame con color rojo detectado", frame_con_color_rojo)

  if cv2.waitKey(1) & 0xFF == ord("q"):
    break

cap.release()
cv2.destroyAllWindows()
