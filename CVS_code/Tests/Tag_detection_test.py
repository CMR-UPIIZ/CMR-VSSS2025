#import cv2
#import numpy as np
#
#cap = cv2.VideoCapture(1)
#
## Leer la imagen y el modelo de tag conocido
##imagen = cv2.imread('imagen_con_tags.jpg')
#ret,imagen = cap.read()
#modelo_tag = cv2.imread('Y_RG_code_1.png')
#
## Convertir la imagen a escala de grises
#gray_imagen = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
#gray_modelo = cv2.cvtColor(modelo_tag, cv2.COLOR_BGR2GRAY)
#
## Crear el detector ORB
#Moded_detector = cv2.FastFeatureDetector()#cv2.ORB_create()
#
## Encontrar los puntos clave y los descriptores en la imagen y el modelo
#keypoints_imagen, descriptors_imagen = Moded_detector.detect(gray_imagen, None)
#keypoints_modelo, descriptors_modelo = Moded_detector.detectAndCompute(gray_modelo, None)
#
## Emparejar los descriptores entre la imagen y el modelo
#bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#matches = bf.match(descriptors_modelo, descriptors_imagen)
#
## Ordenar las coincidencias por distancia
#matches = sorted(matches, key=lambda x: x.distance)
#
## Dibujar los primeros N emparejamientos (puedes ajustar este valor)
#N = 10
#imagen_match = cv2.drawMatches(modelo_tag, keypoints_modelo, imagen, keypoints_imagen, matches[:N], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
#
## Mostrar la imagen con los emparejamientos
#cv2.imshow('Detección de Tags', imagen_match)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

#import cv2
#import numpy as np
#
## Inicializar el capturador de la webcam
#cap = cv2.VideoCapture(1)
#
## Leer el modelo de tag conocido
#modelo_tag = cv2.imread('Y_RG_code_1.png')
#
## Convertir el modelo de tag a escala de grises
#gray_modelo = modelo_tag#cv2.cvtColor(modelo_tag, cv2.COLOR_BGR2GRAY)
#
## Crear el detector FAST
#fast = cv2.ORB_create()#cv2.FastFeatureDetector_create()
#
#while True:
#  # Capturar un frame de la webcam
#  ret, frame = cap.read()
#  if not ret:
#    break
#  
#  # Convertir el frame a escala de grises
#  gray_frame = frame#cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#
#  # Detectar puntos clave en el frame
#  #keypoints_frame = fast.detect(gray_frame, None)
#  #  # Extraer descriptores del modelo de tag
#  #kp_modelo = fast.detect(gray_modelo, None)
#
#  ## Extraer descriptores del frame
#  #des_frame = np.array([keypoint.response for keypoint in keypoints_frame])
#  #
#  #des_modelo = np.array([keypoint.response for keypoint in kp_modelo])
#  keypoints_frame, des_frame = fast.detectAndCompute(gray_frame, None)
#  kp_modelo, des_modelo = fast.detectAndCompute(gray_modelo, None)
#
#
#
#  # Emparejar descriptores entre el frame y el modelo
#  bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
#  matches = bf.match(des_modelo, des_frame)
#
#  # Dibujar los primeros N emparejamientos (puedes ajustar este valor)
#  N = 10
#  matches = sorted(matches, key=lambda x: x.distance)
#  imagen_match = cv2.drawMatches(modelo_tag, kp_modelo, frame, keypoints_frame, matches[:N], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
#
#  # Mostrar la imagen con los emparejamientos
#  cv2.imshow('Detección de Tags', imagen_match)
#
#  # Salir del bucle cuando se presiona 'q'
#  if cv2.waitKey(1) & 0xFF == ord('q'):
#      break
#
## Liberar el capturador y cerrar las ventanas
#cap.release()
#cv2.destroyAllWindows()
import cv2, numpy as np

img1 = None
win_name = 'Camera Matching'
MIN_MATCH = 5
# ORB 검출기 생성  ---①
detector = cv2.ORB_create(1000)
# Flann 추출기 생성 ---②
FLANN_INDEX_LSH = 6
index_params= dict( algorithm = FLANN_INDEX_LSH,
                    table_number = 6,
                    key_size = 12,
                    multi_probe_level = 1)
search_params=dict(checks=32)
matcher = cv2.FlannBasedMatcher(index_params, search_params)
# 카메라 캡쳐 연결 및 프레임 크기 축소 ---③
cap = cv2.VideoCapture(1)              
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while cap.isOpened():       
  ret, frame = cap.read() 
  if img1 is None:  # 등록된 이미지 없음, 카메라 바이패스
    res = frame
  else:             # 등록된 이미지 있는 경우, 매칭 시작
    img2 = frame
    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    # 키포인트와 디스크립터 추출
    kp1, desc1 = detector.detectAndCompute(gray1, None)
    kp2, desc2 = detector.detectAndCompute(gray2, None)
    # k=2로 knnMatch
    matches = matcher.knnMatch(desc1, desc2, 2)
    # 이웃 거리의 75%로 좋은 매칭점 추출---②
    ratio = 0.75
    good_matches = [m[0] for m in matches \
                        if len(m) == 2 and m[0].distance < m[1].distance * ratio]
    print('good matches:%d/%d' %(len(good_matches),len(matches)))
    # 모든 매칭점 그리지 못하게 마스크를 0으로 채움
    matchesMask = np.zeros(len(good_matches)).tolist()
    # 좋은 매칭점 최소 갯수 이상 인 경우
    if len(good_matches) > MIN_MATCH: 
      # 좋은 매칭점으로 원본과 대상 영상의 좌표 구하기 ---③
      src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_matches ])
      dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ])
      # 원근 변환 행렬 구하기 ---⑤
      mtrx, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
      accuracy=float(mask.sum()) / mask.size
      print("accuracy: %d/%d(%.2f%%)"% (mask.sum(), mask.size, accuracy))
      if mask.sum() > MIN_MATCH:  # 정상치 매칭점 최소 갯수 이상 인 경우
        # 이상점 매칭점만 그리게 마스크 설정
        matchesMask = mask.ravel().tolist()
        # 원본 영상 좌표로 원근 변환 후 영역 표시  ---⑦
        h,w, = img1.shape[:2]
        pts = np.float32([ [[0,0]],[[0,h-1]],[[w-1,h-1]],[[w-1,0]] ])
        dst = cv2.perspectiveTransform(pts,mtrx)
        img2 = cv2.polylines(img2,[np.int32(dst)],True,255,3, cv2.LINE_AA)
    # 마스크로 매칭점 그리기 ---⑨
    res = cv2.drawMatches(  img1, kp1, img2, kp2, good_matches, None, \
                            matchesMask=matchesMask,
                            flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
  # 결과 출력
  cv2.imshow(win_name, res)
  key = cv2.waitKey(1)
  if key == 27:    # Esc, 종료
    break          
  elif key == ord(' '): # 스페이스 바로 ROI 설정해서 img1 설정
    x,y,w,h = cv2.selectROI(win_name, frame, False)
    if w and h:
      img1 = frame[y:y+h, x:x+w]
else:
  print("can't open camera.")
cap.release()                          
cv2.destroyAllWindows()