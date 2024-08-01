import cv2 as cv
import numpy as np
import math

path_image_folder = "Calib_images/"

lower_colors = {
  'red': np.array([170, 134, 197]),
  'cyan': np.array([85, 100, 100]),
  'green': np.array([80,  87, 171]),
  'magenta': np.array([140, 100, 100]),
  'yellow': np.array([30, 142, 218]),
  'blue': np.array([117, 84, 236])
}
upper_colors = {
  'red': np.array([175, 158, 219]),
  'cyan': np.array([105, 255, 255]),
  'green': np.array([85, 121, 185]),
  'magenta': np.array([170, 255, 255]),
  'yellow': np.array([34, 168, 228]),
  'blue': np.array([125, 102, 255])
}
enableColorDetection = True
enableCalibColor = False
areaThreshold = 100

debug_draws ={
  'centers': True,
  'd_color_rect': False,
  'tag_center':True,
  'tag_perimeter':True,
  'tag_t_lines': False,
  'tag_d_area': False,
  'tag_ID_text': True,
  'angle_ref': True
}

tag_yellow_ID = {
  'ygr':1,
  'ycr':2,
  'yrg':3,
  'ycg':4,
  'ymg':5,
  'yrc':6,
  'ygc':7,
  'ymc':8,
  'ygm':9,
  'ycm':10
}

tag_blue_ID = {
  'bgr':1,
  'bcr':2,
  'brg':3,
  'bcg':4,
  'bmg':5,
  'brc':6,
  'bgc':7,
  'bmc':8,
  'bgm':9,
  'bcm':10
}

frame = cv.imread(path_image_folder + 'f1_calib.png')
ang = float (input('Introduce angulo:'))
h,w = frame.shape[:2]
cn = (w//2,h//2)
rot_m = cv.getRotationMatrix2D(cn,ang,1.0)
frame = cv.warpAffine(frame,rot_m, (w,h))


def find_aprox_square(contorno):
  # Aproximar el contorno a un polígono
  approx = cv.approxPolyDP(contorno, 0.04 * cv.arcLength(contorno, True), True)

  # Encontrar el rectángulo delimitador del polígono aproximado
  rect = cv.boundingRect(approx)

  return rect

# Función para encontrar el centro de un rectángulo
def get_square_center(rect):
  x, y, w, h = rect
  centro_x = x + (w // 2)
  centro_y = y + (h // 2)
  return (centro_x, centro_y)

# Función para encontrar el ángulo de rotación de un rectángulo
def get_square_angule(contorno):
  # Ajustar un rectángulo rotado al contorno
  rect = cv.minAreaRect(contorno)
  _, _, angulo = rect
  return angulo

def hsv_to_rgb(hsv):
  h, s, v = hsv
  h = float(h) / 180.0
  s = float(s) / 255.0
  v = float(v) / 255.0

  if s == 0:
    r = g = b = v
  else:
    i = int(h * 6.0)
    f = (h * 6.0) - i
    p = v * (1.0 - s)
    q = v * (1.0 - s * f)
    t = v * (1.0 - s * (1.0 - f))
    if i % 6 == 0:
      r, g, b = v, t, p
    elif i == 1:
      r, g, b = q, v, p
    elif i == 2:
      r, g, b = p, v, t
    elif i == 3:
      r, g, b = p, q, v
    elif i == 4:
      r, g, b = t, p, v
    else:
      r, g, b = v, p, q

  return [int(r * 255), int(g * 255), int(b * 255)]

def doColorID(frame):
  # Convertir de BGR a HSV
  hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

  # Seleccionar un área en la imagen
  (x, y, w, h) = cv.selectROI('ColorPicker', frame, fromCenter=False, showCrosshair=True)
  cv.destroyWindow('ColorPicker')

  # Recortar el área seleccionada
  selected_area = hsv[y:y+h, x:x+w]

  # Calcular los vectores de color HSV promedio, más opaco y más brillante
  prom_color = np.round(np.mean(selected_area, axis=(0, 1))).astype(int)
  opaque_color = np.min(selected_area, axis=(0, 1)).astype(int)
  shine_color = np.max(selected_area, axis=(0, 1)).astype(int)

  # Imprimir los resultados
  prom_color_rgb = hsv_to_rgb(prom_color)
  opaque_color_rgb = hsv_to_rgb(opaque_color)
  shine_color_rgb = hsv_to_rgb(shine_color)
  prom_color_hex = '#{:02x}{:02x}{:02x}'.format(prom_color_rgb[0], 
                                                prom_color_rgb[1],
                                                prom_color_rgb[2])
  opaque_color_hex = '#{:02x}{:02x}{:02x}'.format( opaque_color_rgb[0], 
                                                      opaque_color_rgb[1],
                                                      opaque_color_rgb[2])
  shine_color_hex = '#{:02x}{:02x}{:02x}'.format( shine_color_rgb[0], 
                                                          shine_color_rgb[1],
                                                          shine_color_rgb[2])
  #-debug
  print("Color promedio (H, S, V):", prom_color)
  print("Color más opaco (H, S, V):", opaque_color)
  print("Color más brillante (H, S, V):", shine_color)

def get_distance_btwn_points(A,B):
  x1, y1 = A
  x2, y2 = B

  d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
  return d

def get_horiz_inclination(A,B):
  x1,y1 = A
  x2,y2 = B
  
  y1 = -y1
  y2 = -y2 

  y_discr = (y2 - y1)
  x_discr = (x2 - x1)

  if(abs(x_discr) == 0): #poinst aliend vertically
    if y_discr >= 0 :
      theta_deg = 90
    else:
      theta_deg = -90
  elif(abs(y_discr) == 0):
    if x_discr >= 0 :
      theta_deg = 0
    else:
      theta_deg = 180
  else:
    m = (y2-y1)/(x2-x1)
    theta = math.atan(m)
    theta_deg = math.degrees(theta)

    if ((x_discr<0) and (y_discr<0)):
      theta_deg = -180 + theta_deg
    elif ((x_discr<0) and (y_discr>0)):
      theta_deg = 180 + theta_deg

  return theta_deg

def get_vert_inclination(A,B):
  x1,y1 = A
  x2,y2 = B

  y_discr = (y2 - y1)
  x_discr = (x2 - x1)


  if(abs(x_discr) <= 2): #poinst aliend vertically
    if y_discr >= 0 :
      theta_deg = 0
    else:
      theta_deg = 180
  elif(abs(y_discr) == 0): #poinst aliend horiz
    if x_discr >= 0 :
      theta_deg = -90
    else:
      theta_deg = 90
  else:
    m = (y2-y1)/(x2-x1)
    theta = math.atan(m)
    theta_deg = math.degrees(theta)

    if ((x_discr>0) and (y_discr>0)):
      theta_deg = -(90 + theta_deg)
    elif ((x_discr<0) and (y_discr<0)):
      theta_deg = 90 - theta_deg
    elif ((x_discr<0) and (y_discr>0)):
      theta_deg = 90 - theta_deg
    elif ((x_discr>0) and (y_discr<0)):
      theta_deg = -90 - theta_deg
  
  return theta_deg

def draw_rotated_rectangle(image, center, side_length, angle):
  # Calcular la mitad del lado para obtener las coordenadas del rectángulo sin rotar
  half_side = side_length / 2

  # Crear una matriz de puntos del rectángulo sin rotar (en el orden: top-left, top-right, bottom-right, bottom-left)
  rect_points = np.array([
      [-half_side, -half_side],
      [half_side, -half_side],
      [half_side, half_side],
      [-half_side, half_side]
  ])

  # Crear la matriz de rotación 2D
  rotation_matrix = cv.getRotationMatrix2D((0, 0), angle, 1.0)

  # Aplicar la rotación a cada punto
  rotated_points = np.dot(rect_points, rotation_matrix[:, :2].T).astype(int)

  # Trasladar los puntos al centro especificado
  rotated_points += np.array(center)

  # Dibujar el rectángulo rotado
  cv.polylines(image, [rotated_points], isClosed=True, color=(237,107,213), thickness=2)


def detec_tag(frame):
  list_yellow_rad = []
  list_blue_rad = []

  tag_main_y = []
  tag_main_b = []

  list_centers_Yellow = []
  list_centers_Blue = []
  list_centers_Red = []
  list_centers_Green = []
  list_centers_Cyan = []
  list_centers_Magenta = []

  # BGR TO HSV CONVERSION
  hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

  # CREATE A MASK FOR COLORS
  mask_yel = cv.inRange(hsv, lower_colors['yellow'], upper_colors['yellow'])
  mask_blue = cv.inRange(hsv, lower_colors['blue'], upper_colors['blue'])
  mask_red = cv.inRange(hsv, lower_colors['red'], upper_colors['red'])
  mask_green = cv.inRange(hsv, lower_colors['green'], upper_colors['green'])
  mask_cyan = cv.inRange(hsv, lower_colors['cyan'], upper_colors['cyan'])
  mask_magenta = cv.inRange(hsv, lower_colors['magenta'], upper_colors['magenta'])
  # ------ falta el naranga de la pelota

  # Find contors of masks
  contoursYellow, _ = cv.findContours(mask_yel, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
  contoursBlue, _ = cv.findContours(mask_blue, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
  contoursRed, _ = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
  contoursGreen, _ = cv.findContours(mask_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
  contoursCyan, _ = cv.findContours(mask_cyan, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
  contoursMagenta, _ = cv.findContours(mask_magenta, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
  

  #contours = contoursRed + contoursYellow

  # CONTOURS
  # --- for Yellow
  for contour in contoursYellow:
    area = cv.contourArea(contour)
    if area > areaThreshold:
      #approx = cv.approxPolyDP(contour, 0.04 * cv.arcLength(contour, True), True)
      rect = find_aprox_square(contour)
      ang = get_square_angule(contour)
      #print(f'angule Yellow square = {ang}')
      box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), 0))
      box = np.intp(box)

      c_pos = get_square_center(rect)
      r = int(math.sqrt(rect[2]**2 + rect[3]**2) * 0.68)

      if debug_draws['d_color_rect']: 
        cv.drawContours(frame, [box], 0, (0, 255, 255), 2)
      if debug_draws['centers']:
        cv.circle(frame, c_pos, 5, (0, 255, 255), -1)
      
      if debug_draws['tag_d_area']:
        cv.circle(frame, c_pos, r , (0, 255, 255), 1)

      list_yellow_rad.append(r)
      list_centers_Yellow.append(c_pos)

  # --- for Blue
  for contour in contoursBlue:
    area = cv.contourArea(contour)
    if area > areaThreshold:
      #approx = cv.approxPolyDP(contour, 0.04 * cv.arcLength(contour, True), True)
      rect = find_aprox_square(contour)
      ang = get_square_angule(contour)
      #print(f'angule Yellow square = {ang}')
      box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), 0))
      box = np.intp(box)

      c_pos = get_square_center(rect)
      r = int(math.sqrt(rect[2]**2 + rect[3]**2) * 0.68)

      if debug_draws['d_color_rect']: 
        cv.drawContours(frame, [box], 0, (255, 0, 0), 2)
      if debug_draws['centers']:
        cv.circle(frame, c_pos, 5, (255, 0, 0), -1)
      
      if debug_draws['tag_d_area']:
        cv.circle(frame, c_pos, r , (255, 0, 0), 1)

      list_blue_rad.append(r)
      list_centers_Blue.append(c_pos)

  # --- for Red
  for contour in contoursRed:
    area = cv.contourArea(contour)
    if area > areaThreshold:
      rect = find_aprox_square(contour)
      ang = get_square_angule(contour)
      box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), ang))
      box = np.intp(box)

      c_pos = get_square_center(rect)

      if debug_draws['d_color_rect']:
        cv.drawContours(frame, [box], 0, (0, 0, 255), 2)
      if debug_draws['centers']:
        cv.circle(frame, c_pos, 5, (0, 0, 255), -1)

      list_centers_Red.append(c_pos)
  
  # --- for Green
  for contour in contoursGreen:
    area = cv.contourArea(contour)
    if area > areaThreshold:
      rect = find_aprox_square(contour)
      ang = get_square_angule(contour)

      box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), ang))
      box = np.intp(box)

      c_pos = get_square_center(rect)
      
      if debug_draws['d_color_rect']:
        cv.drawContours(frame, [box], 0, (0, 255, 0), 2)
      if debug_draws['centers']:
        cv.circle(frame, c_pos, 5, (0, 255, 0), -1)
      
      list_centers_Green.append(c_pos)

  # --- for Cyan
  for contour in contoursCyan:
    area = cv.contourArea(contour)
    if area > areaThreshold:
      rect = find_aprox_square(contour)
      ang = get_square_angule(contour)

      box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), ang))
      box = np.intp(box)

      c_pos = get_square_center(rect)
      
      if debug_draws['d_color_rect']:
        cv.drawContours(frame, [box], 0, (235, 233, 110), 2)
      if debug_draws['centers']:
        cv.circle(frame, c_pos, 5, (235, 233, 110), -1)
      
      list_centers_Cyan.append(c_pos)

  # --- for Magenta
  for contour in contoursMagenta:
    area = cv.contourArea(contour)
    if area > areaThreshold:
      rect = find_aprox_square(contour)
      ang = get_square_angule(contour)

      box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), ang))
      box = np.intp(box)

      c_pos = get_square_center(rect)
      
      if debug_draws['d_color_rect']:
        cv.drawContours(frame, [box], 0, (247, 69, 247), 2)
      if debug_draws['centers']:
        cv.circle(frame, c_pos, 5, (247, 69, 247), -1)
      
      list_centers_Green.append(c_pos)


  # Detect tag 
  if len(list_centers_Yellow) >0:
    for i in range(len(list_centers_Yellow)):
      t_i = 1
      center_yel = list_centers_Yellow[i]
      t_ps = [center_yel] #Point of the centers of colors
      t_ps_d = [0]        #Distances of the center second color to the main color
      t_ps_c = ['y']      #Code of de color

      #-> RED color association identification
      vaild_p_d = 1000000
      valid_p = None
      for center_red in list_centers_Red:
        D = get_distance_btwn_points(center_yel,center_red)
        if D < list_yellow_rad[i]:
          if(D < vaild_p_d):
            vaild_p_d = D
            valid_p = center_red
      
      if valid_p != None:
        t_i += 1
        t_ps.append(valid_p)
        t_ps_d.append(vaild_p_d)
        t_ps_c.append('r')

      #-> GREEN color association identification
      vaild_p_d = 1000000
      valid_p = None
      for center_green in list_centers_Green:
        D = get_distance_btwn_points(center_yel,center_green)
        if D < list_yellow_rad[i]:
          if(D < vaild_p_d):
            vaild_p_d = D
            valid_p = center_green

      if valid_p != None:
        t_i += 1
        t_ps.append(valid_p)
        t_ps_d.append(vaild_p_d)
        t_ps_c.append('g')
      
      # - CYAN color association identification 
      vaild_p_d = 1000000
      valid_p = None
      for center_cyan in list_centers_Cyan:
        D = get_distance_btwn_points(center_yel,center_cyan)
        if D < list_yellow_rad[i]:
          if(D < vaild_p_d):
            vaild_p_d = D
            valid_p = center_cyan
      
      if valid_p != None:
        t_i += 1
        t_ps.append(valid_p)
        t_ps_d.append(vaild_p_d)
        t_ps_c.append('c')
    
      #-> MAGENT color association identification 
      vaild_p_d = 1000000
      valid_p = None 
      for center_magent in list_centers_Magenta:
        D = get_distance_btwn_points(center_yel,center_magent)
        if D < list_yellow_rad[i]:
          if(D < vaild_p_d):
            vaild_p_d = D
            valid_p = center_magent
      
      if valid_p != None:
        t_i += 1
        t_ps.append(valid_p)
        t_ps_d.append(vaild_p_d)
        t_ps_c.append('m')

      #-> Find the possible colors for the TAG
      if t_i > 3:
        min_d_point  = ()
        pre_min_d_point = ()
        min_d_p_code = ''
        pre_mid_d_p_code = ''
        aux_d = 1000000
        for j in range(len(t_ps_d)):
          if j == 0:
            pass
          else:
            D = t_ps_d[i]
            if D < aux_d:
              aux_d = D
              pre_min_d_point = min_d_point
              pre_mid_d_p_code = min_d_p_code
              min_d_point = t_ps[i]
              min_d_p_code = t_ps_c[i]
      
        t_ps = [center_yel, min_d_point, pre_min_d_point]
        t_ps_c = ['y',min_d_p_code, pre_mid_d_p_code]
        t_i = 3

      #-> Find TAG parameters
      if t_i == 3 :
        if debug_draws['tag_t_lines']:
          cv.line(frame,t_ps[0],t_ps[1],(255,0,0),2)
          cv.line(frame,t_ps[0],t_ps[2],(255,0,0),2)
          cv.line(frame,t_ps[1],t_ps[2],(107,237,170),3)
        
        x1,y1 = t_ps[0]
        x2,y2 = t_ps[1]
        x3,y3 = t_ps[2]

        c = ((x1+x2+x3)//3,(y1+y2+y3)//3)
        #d_m = get_distance_btwn_points(c,t_ps[0])
        d_a_deg = get_vert_inclination(t_ps[0],c)
        d_a = math.radians(d_a_deg)
        d_p = (c[0] - int(list_yellow_rad[i]*0.8*math.sin(d_a)), c[1] - int(list_yellow_rad[i]*0.8*math.cos(d_a)))
        #d_pp = (c[0] - int(d_m*math.sin(d_a +3.1416)), c[1] - int(d_m*math.cos(d_a+3.1416)))

        if debug_draws['tag_center']:
          cv.circle(frame, c, 5, (237,107,213), -1)
        if debug_draws['tag_perimeter']:
          cv.line(frame, c, d_p, (237,107,213), 2)
          #cv.line(frame, c, d_pp,(0,255,0), 2 )
          draw_rotated_rectangle(frame,c,list_yellow_rad[i]*1.7,d_a_deg)
        if debug_draws['angle_ref']:
          cv.line(frame, c,(c[0],c[1]-20),(0,0,213),2)

        # TAG ID Identify
        ID_found = 'y'
        if (d_a_deg<= 40 and d_a_deg >= -40):
          if(x2 < x3):
            ID_found += t_ps_c[1] + t_ps_c[2]
          else:
            ID_found += t_ps_c[2] + t_ps_c[1]
        elif (d_a_deg > 40 and d_a_deg < 120):
          if(y2 > y3):
            ID_found += t_ps_c[1] + t_ps_c[2]
          else:
            ID_found += t_ps_c[2] + t_ps_c[1]
        elif (d_a_deg < -40 and d_a_deg > -120):
          if(y3 > y2):
            ID_found += t_ps_c[1] + t_ps_c[2]
          else:
            ID_found += t_ps_c[2] + t_ps_c[1]
        else: # 120->180 and -120 -> -
          if(x2 > x3):
            ID_found += t_ps_c[1] + t_ps_c[2]
          else:
            ID_found += t_ps_c[2] + t_ps_c[1]
          pass

        if debug_draws['tag_ID_text']:
          t = f'Y-TAG-{tag_yellow_ID[ID_found]}'
          cv.putText(frame, t, (c[0]-30,c[1]+20), cv.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,0),2)

        t_ps.append(c)
        tag_main_y.append(t_ps)

        print(f"Se detecto un Tag!! \nCon centro en: {c}\nY una inclinazion resperto a la vertical de {round(d_a_deg)}°")
        print(f"El ID es '{ID_found}' del TAG {tag_yellow_ID[ID_found]}")

  if len(list_centers_Blue) > 0:
    for i in range(len(list_centers_Blue)):
      t_i = 1
      center_blue = list_centers_Blue[i]
      t_ps = [center_blue]#Point of the centers of colors
      t_ps_d = [0]        #Distances of the center second color to the main color
      t_ps_c = ['b']      #Code of de color

      #-> RED color association identification
      vaild_p_d = 1000000
      valid_p = None
      for center_red in list_centers_Red:
        D = get_distance_btwn_points(center_blue,center_red)
        if D < list_blue_rad[i]:
          if(D < vaild_p_d):
            vaild_p_d = D
            valid_p = center_red
      
      if valid_p != None:
        t_i += 1
        t_ps.append(valid_p)
        t_ps_d.append(vaild_p_d)
        t_ps_c.append('r')

      #-> GREEN color association identification
      vaild_p_d = 1000000
      valid_p = None
      for center_green in list_centers_Green:
        D = get_distance_btwn_points(center_blue,center_green)
        if D < list_blue_rad[i]:
          if(D < vaild_p_d):
            vaild_p_d = D
            valid_p = center_green

      if valid_p != None:
        t_i += 1
        t_ps.append(valid_p)
        t_ps_d.append(vaild_p_d)
        t_ps_c.append('g')
      
      # - CYAN color association identification 
      vaild_p_d = 1000000
      valid_p = None
      for center_cyan in list_centers_Cyan:
        D = get_distance_btwn_points(center_blue,center_cyan)
        if D < list_blue_rad[i]:
          if(D < vaild_p_d):
            vaild_p_d = D
            valid_p = center_cyan
      
      if valid_p != None:
        t_i += 1
        t_ps.append(valid_p)
        t_ps_d.append(vaild_p_d)
        t_ps_c.append('c')
    
      #-> MAGENT color association identification 
      vaild_p_d = 1000000
      valid_p = None 
      for center_magent in list_centers_Magenta:
        D = get_distance_btwn_points(center_blue,center_magent)
        if D < list_blue_rad[i]:
          if(D < vaild_p_d):
            vaild_p_d = D
            valid_p = center_magent
      
      if valid_p != None:
        t_i += 1
        t_ps.append(valid_p)
        t_ps_d.append(vaild_p_d)
        t_ps_c.append('m')

      #-> Find the possible colors for the TAG
      if t_i > 3:
        min_d_point  = ()
        pre_min_d_point = ()
        min_d_p_code = ''
        pre_mid_d_p_code = ''
        aux_d = 1000000
        for j in range(len(t_ps_d)):
          if j == 0:
            pass
          else:
            D = t_ps_d[i]
            if D < aux_d:
              aux_d = D
              pre_min_d_point = min_d_point
              pre_mid_d_p_code = min_d_p_code
              min_d_point = t_ps[i]
              min_d_p_code = t_ps_c[i]
      
        t_ps = [center_blue, min_d_point, pre_min_d_point]
        t_ps_c = ['b',min_d_p_code, pre_mid_d_p_code]
        t_i = 3

      #-> Find TAG parameters
      if t_i == 3 :
        if debug_draws['tag_t_lines']:
          cv.line(frame,t_ps[0],t_ps[1],(255,0,0),2)
          cv.line(frame,t_ps[0],t_ps[2],(255,0,0),2)
          cv.line(frame,t_ps[1],t_ps[2],(107,237,170),3)
        
        x1,y1 = t_ps[0]
        x2,y2 = t_ps[1]
        x3,y3 = t_ps[2]

        c = ((x1+x2+x3)//3,(y1+y2+y3)//3)
        #d_m = get_distance_btwn_points(c,t_ps[0])
        d_a_deg = get_vert_inclination(t_ps[0],c)
        d_a = math.radians(d_a_deg)
        d_p = (c[0] - int(list_blue_rad[i]*0.8*math.sin(d_a)), c[1] - int(list_blue_rad[i]*0.8*math.cos(d_a)))
        #d_pp = (c[0] - int(d_m*math.sin(d_a +3.1416)), c[1] - int(d_m*math.cos(d_a+3.1416)))

        if debug_draws['tag_center']:
          cv.circle(frame, c, 5, (237,107,213), -1)
        if debug_draws['tag_perimeter']:
          cv.line(frame, c, d_p, (237,107,213), 2)
          #cv.line(frame, c, d_pp,(0,255,0), 2 )
          draw_rotated_rectangle(frame,c,list_blue_rad[i]*1.7,d_a_deg)
        if debug_draws['angle_ref']:
          cv.line(frame, c,(c[0],c[1]-20),(0,0,213),2)

        # TAG ID Identify
        ID_found = 'b'
        if (d_a_deg<= 40 and d_a_deg >= -40):
          if(x2 < x3):
            ID_found += t_ps_c[1] + t_ps_c[2]
          else:
            ID_found += t_ps_c[2] + t_ps_c[1]
        elif (d_a_deg > 40 and d_a_deg < 120):
          if(y2 > y3):
            ID_found += t_ps_c[1] + t_ps_c[2]
          else:
            ID_found += t_ps_c[2] + t_ps_c[1]
        elif (d_a_deg < -40 and d_a_deg > -120):
          if(y3 > y2):
            ID_found += t_ps_c[1] + t_ps_c[2]
          else:
            ID_found += t_ps_c[2] + t_ps_c[1]
        else: # 120->180 and -120 -> -
          if(x2 > x3):
            ID_found += t_ps_c[1] + t_ps_c[2]
          else:
            ID_found += t_ps_c[2] + t_ps_c[1]
          pass

        if debug_draws['tag_ID_text']:
          t = f'B-TAG-{tag_blue_ID[ID_found]}'
          cv.putText(frame, t, (c[0]-30,c[1]+20), cv.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,0),2)

        t_ps.append(c)
        tag_main_b.append(t_ps)

        print(f"Se detecto un Tag!! \nCon centro en: {c}\nY una inclinazion resperto a la vertical de {round(d_a_deg)}°")
        print(f"El ID es '{ID_found}' del TAG {tag_blue_ID[ID_found]}")

  return frame



if frame is not None:
  print("Se leyo imagen correctamente")

  if enableCalibColor:
    doColorID(frame)
  
  detec_tag(frame)
  
  while True:
    cv.imshow('Imagen', frame)
    key = cv.waitKey(1) & 0xFF
    # Salir si se presiona 'q'
    if key == ord("q"):
      break
else:
  print("ERROR al leer imagen")