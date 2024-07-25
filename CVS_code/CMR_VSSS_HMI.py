import cv2 as cv
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import json
import numpy as np
import math

path_image_folder = "Tests/Calib_images/"
path_camera_parameter = "Camera_Cali_parameters/"
path_tag_color_calib_file = "ColorTags_VSSS/Color_Range_Calib_Values.json"

class WebCamParameters:
  def __init__(self):
    self.available_cameras = self.get_available_cameras()
    self.selected_camera_index = None
    self.webcam = None
    self.photo_counter = 1
    self.camera_size = ()
    self.camera_status = False
    self.current_frame = None

    self.current_px_position = None
    self.event_px_pos = False
    self.HGP_point_values = [None,None,None,None]
    self.enableHGP_videoTrasf = False

    self.Tcolors_LowRange = {}
    self.Tcolors_HighRange = {}
    self.Tcolors_Average = {}
    self.enableColorDetection = False
    self.current_color_picked = [None,None,None] #in color space HSV

    self.debug_draws = {
      'centers': True,
      'd_color_rect': False,
      'tag_center':True,
      'tag_perimeter':True,
      'tag_t_lines': False,
      'tag_d_area': False,
      'tag_ID_text': True,
      'angle_ref': False
    }
    self.areaThreshold = 100

    self.tag_yellow_ID = {
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
    self.tag_blue_ID = {
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

    self.loadParameters()

  def get_available_cameras(self):
    camera_list = []
    for i in range(5):
      cam = cv.VideoCapture(i,cv.CAP_DSHOW)
      if cam.isOpened():
        camera_list.append(f'Camera {i}')
        cam.release()
    return camera_list
  
  def connect_camera(self):
      self.webcam = cv.VideoCapture(self.selected_camera_index, cv.CAP_DSHOW)
      if not self.webcam.isOpened():
        print(f"Error: No se pudo abrir la cámara {self.selected_camera_index}")
      else:
        self.camera_status = True
        self.camera_size = (int(self.webcam.get(cv.CAP_PROP_FRAME_WIDTH)),
                            int(self.webcam.get(cv.CAP_PROP_FRAME_HEIGHT)))

  def disconnect_camera(self):
    if self.webcam is not None:
      self.webcam.release()
      self.webcam = None
      self.camera_status = False
  
  def setCameraIndex(self, index):
    if index < len(self.available_cameras):
      self.selected_camera_index = index
    else:
      print(f'Error la camara index {index}')
  
  def set_px_position(self, x,y):
    self.current_px_position = (x,y)
    self.event_px_pos = True
  
  def get_px_position(self):
    if self.current_px_position is not None:
      self.event_px_pos = False
    return self.current_px_position

  def check_valid_HGPpoits(self):
    result = False
    for i in range(4):
      if self.HGP_point_values[i] is not None:
        result = True
    return result

  def check_valid_ColorPick(self):
    result = False
    for i in range(len(self.current_color_picked)):
      if self.current_color_picked[i] is None:
        result = False
      else:
        result = True
    return result

  def loadParameters(self):
    try:
      with open(path_tag_color_calib_file,'r') as f:
        data =  json.load(f)
        self.Tcolors_LowRange = data["Tcolors_LowRange"]
        self.Tcolors_HighRange = data["Tcolors_HighRange"]
        self.Tcolors_Average = data["Tcolors_Average"]
      print("Tag Color Calibration Parameters LOAD Succesfully")
    except Exception as e:
      print(f"ERROR Tag Color Calibration Parameters DID NOT LOAD:\n{e}")

  def saveParameters(self):
    data = {
      "Tcolors_LowRange":self.Tcolors_LowRange,
      "Tcolors_HighRange":self.Tcolors_HighRange,
      "Tcolors_Average":self.Tcolors_Average
    }
    
    try:
      with open(path_tag_color_calib_file,'w') as f:
        json.dump(data,f,default=self.numpy_array_to_list)
      print("Tag Color Calibration Parameters Saved Succesfully")
    except Exception as e:
      print(f"ERROR Tag Color Calibration Parameters DID NOT SAVE:\n{e}")

  @staticmethod
  def numpy_array_to_list(obj):
    if isinstance(obj, np.ndarray):
      if obj.dtype == np.uint8:
        return obj.astype(int).tolist()
      return obj.tolist()
    elif isinstance(obj, list):
      return obj
    raise TypeError(f'Object of type {type(obj)} is not JSON serializable')

  @staticmethod
  def find_aprox_square(contorno):
    # Aproximar el contorno a un polígono
    approx = cv.approxPolyDP(contorno, 0.04 * cv.arcLength(contorno, True), True)

    # Encontrar el rectángulo delimitador del polígono aproximado
    rect = cv.boundingRect(approx)

    return rect

  # Función para encontrar el centro de un rectángulo
  @staticmethod
  def get_square_center(rect):
    x, y, w, h = rect
    centro_x = x + (w // 2)
    centro_y = y + (h // 2)
    return (centro_x, centro_y)

  # Función para encontrar el ángulo de rotación de un rectángulo
  @staticmethod
  def get_square_angule(contorno):
    # Ajustar un rectángulo rotado al contorno
    rect = cv.minAreaRect(contorno)
    _, _, angulo = rect
    return angulo

  @staticmethod
  def get_distance_btwn_points(A,B):
    x1, y1 = A
    x2, y2 = B

    d = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return d

  @staticmethod
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

  @staticmethod
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

  @staticmethod
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

  def doTagDetection(self, frame):
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
    mask_yel = cv.inRange(hsv, np.array(self.Tcolors_LowRange['Yellow']),
                          np.array(self.Tcolors_HighRange['Yellow']))
    mask_blue = cv.inRange(hsv, np.array(self.Tcolors_LowRange['Blue']),
                            np.array(self.Tcolors_HighRange['Blue']))
    mask_red = cv.inRange(hsv, np.array(self.Tcolors_LowRange['Red']),
                          np.array(self.Tcolors_HighRange['Red']))
    mask_green = cv.inRange(hsv, np.array(self.Tcolors_LowRange['Green']),
                            np.array(self.Tcolors_HighRange['Green']))
    mask_cyan = cv.inRange(hsv, np.array(self.Tcolors_LowRange['Cyan']),
                            np.array(self.Tcolors_HighRange['Cyan']))
    mask_magenta = cv.inRange(hsv, np.array(self.Tcolors_LowRange['Magent']),
                              np.array(self.Tcolors_HighRange['Magent']))
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
      if area > self.areaThreshold:
        #approx = cv.approxPolyDP(contour, 0.04 * cv.arcLength(contour, True), True)
        rect = self.find_aprox_square(contour)
        ang = self.get_square_angule(contour)
        #print(f'angule Yellow square = {ang}')
        box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), 0))
        box = np.intp(box)

        c_pos = self.get_square_center(rect)
        r = int(math.sqrt(rect[2]**2 + rect[3]**2) * 0.68)

        if self.debug_draws['d_color_rect']: 
          cv.drawContours(frame, [box], 0, (0, 255, 255), 2)
        if self.debug_draws['centers']:
          cv.circle(frame, c_pos, 5, (0, 255, 255), -1)
        
        if self.debug_draws['tag_d_area']:
          cv.circle(frame, c_pos, r , (0, 255, 255), 1)

        list_yellow_rad.append(r)
        list_centers_Yellow.append(c_pos)

    # --- for Blue
    for contour in contoursBlue:
      area = cv.contourArea(contour)
      if area > self.areaThreshold:
        #approx = cv.approxPolyDP(contour, 0.04 * cv.arcLength(contour, True), True)
        rect = self.find_aprox_square(contour)
        ang = self.get_square_angule(contour)
        #print(f'angule Yellow square = {ang}')
        box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), 0))
        box = np.intp(box)

        c_pos = self.get_square_center(rect)
        r = int(math.sqrt(rect[2]**2 + rect[3]**2) * 0.68)

        if self.debug_draws['d_color_rect']: 
          cv.drawContours(frame, [box], 0, (255, 0, 0), 2)
        if self.debug_draws['centers']:
          cv.circle(frame, c_pos, 5, (255, 0, 0), -1)
        
        if self.debug_draws['tag_d_area']:
          cv.circle(frame, c_pos, r , (255, 0, 0), 1)

        list_blue_rad.append(r)
        list_centers_Blue.append(c_pos)

    # --- for Red
    for contour in contoursRed:
      area = cv.contourArea(contour)
      if area > self.areaThreshold:
        rect = self.find_aprox_square(contour)
        ang = self.get_square_angule(contour)
        box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), ang))
        box = np.intp(box)

        c_pos = self.get_square_center(rect)

        if self.debug_draws['d_color_rect']:
          cv.drawContours(frame, [box], 0, (0, 0, 255), 2)
        if self.debug_draws['centers']:
          cv.circle(frame, c_pos, 5, (0, 0, 255), -1)

        list_centers_Red.append(c_pos)
    
    # --- for Green
    for contour in contoursGreen:
      area = cv.contourArea(contour)
      if area > self.areaThreshold:
        rect = self.find_aprox_square(contour)
        ang = self.get_square_angule(contour)

        box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), ang))
        box = np.intp(box)

        c_pos = self.get_square_center(rect)
        
        if self.debug_draws['d_color_rect']:
          cv.drawContours(frame, [box], 0, (0, 255, 0), 2)
        if self.debug_draws['centers']:
          cv.circle(frame, c_pos, 5, (0, 255, 0), -1)
        
        list_centers_Green.append(c_pos)

    # --- for Cyan
    for contour in contoursCyan:
      area = cv.contourArea(contour)
      if area > self.areaThreshold:
        rect = self.find_aprox_square(contour)
        ang = self.get_square_angule(contour)

        box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), ang))
        box = np.intp(box)

        c_pos = self.get_square_center(rect)
        
        if self.debug_draws['d_color_rect']:
          cv.drawContours(frame, [box], 0, (235, 233, 110), 2)
        if self.debug_draws['centers']:
          cv.circle(frame, c_pos, 5, (235, 233, 110), -1)
        
        list_centers_Green.append(c_pos)

    # --- for Magenta
    for contour in contoursMagenta:
      area = cv.contourArea(contour)
      if area > self.areaThreshold:
        rect = self.find_aprox_square(contour)
        ang = self.get_square_angule(contour)

        box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), ang))
        box = np.intp(box)

        c_pos = self.get_square_center(rect)
        
        if self.debug_draws['d_color_rect']:
          cv.drawContours(frame, [box], 0, (247, 69, 247), 2)
        if self.debug_draws['centers']:
          cv.circle(frame, c_pos, 5, (247, 69, 247), -1)
        
        list_centers_Green.append(c_pos)


    # Detect tag 
    if len(list_centers_Yellow) >0:
      for i in range(len(list_centers_Yellow)):
        t_i = 1
        center_yel = list_centers_Yellow[i]
        t_ps = [center_yel]
        t_ps_c = ['y']

        for center_red in list_centers_Red:
          if self.get_distance_btwn_points(center_yel,center_red) < list_yellow_rad[i]:
            t_i += 1
            t_ps.append(center_red)
            t_ps_c.append('r')

        for center_green in list_centers_Green:
          if self.get_distance_btwn_points(center_yel,center_green) < list_yellow_rad[i]:
            t_i += 1
            t_ps.append(center_green) 
            t_ps_c.append('g')
      
        for center_cyan in list_centers_Cyan:
          if self.get_distance_btwn_points(center_yel,center_cyan) < list_yellow_rad[i]:
            t_i += 1
            t_ps.append(center_cyan) 
            t_ps_c.append('c')
      
        for center_magent in list_centers_Magenta:
          if self.get_distance_btwn_points(center_yel,center_magent) < list_yellow_rad[i]:
            t_i += 1
            t_ps.append(center_magent) 
            t_ps_c.append('m')

        if t_i >= 3 :
          if self.debug_draws['tag_t_lines']:
            cv.line(frame,t_ps[0],t_ps[1],(255,0,0),2)
            cv.line(frame,t_ps[0],t_ps[2],(255,0,0),2)
            cv.line(frame,t_ps[1],t_ps[2],(107,237,170),3)
          
          x1,y1 = t_ps[0]
          x2,y2 = t_ps[1]
          x3,y3 = t_ps[2]

          c = ((x1+x2+x3)//3,(y1+y2+y3)//3)
          #d_m = get_distance_btwn_points(c,t_ps[0])
          d_a_deg = self.get_vert_inclination(t_ps[0],c)
          d_a = math.radians(d_a_deg)
          d_p = (c[0] - int(list_yellow_rad[i]*0.8*math.sin(d_a)), c[1] - int(list_yellow_rad[i]*0.8*math.cos(d_a)))
          #d_pp = (c[0] - int(d_m*math.sin(d_a +3.1416)), c[1] - int(d_m*math.cos(d_a+3.1416)))

          if self.debug_draws['tag_center']:
            cv.circle(frame, c, 5, (237,107,213), -1)
          if self.debug_draws['tag_perimeter']:
            cv.line(frame, c, d_p, (237,107,213), 2)
            #cv.line(frame, c, d_pp,(0,255,0), 2 )
            self.draw_rotated_rectangle(frame,c,list_yellow_rad[i]*1.7,d_a_deg)
          if self.debug_draws['angle_ref']:
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

          if self.debug_draws['tag_ID_text']:
            t = f'Y-TAG-{self.tag_yellow_ID[ID_found]}'
            cv.putText(frame, t, (c[0]-30,c[1]+20), cv.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,0),2)

          t_ps.append(c)
          tag_main_y.append(t_ps)

          print(f"Se detecto un Tag!! \nCon centro en: {c}\nY una inclinazion resperto a la vertical de {round(d_a_deg)}°")
          print(f"El ID es '{ID_found}' del TAG {self.tag_yellow_ID[ID_found]}")

    if len(list_centers_Blue) > 0:
      for i in range(len(list_centers_Blue)):
        t_i = 1
        center_blue = list_centers_Blue[i]
        t_ps = [center_blue]
        t_ps_c = ['b']

        for center_red in list_centers_Red:
          if self.get_distance_btwn_points(center_blue,center_red) < list_blue_rad[i]:
            t_i += 1
            t_ps.append(center_red)
            t_ps_c.append('r')

        for center_green in list_centers_Green:
          if self.get_distance_btwn_points(center_blue,center_green) < list_blue_rad[i]:
            t_i += 1
            t_ps.append(center_green) 
            t_ps_c.append('g')
      
        for center_cyan in list_centers_Cyan:
          if self.get_distance_btwn_points(center_blue,center_cyan) < list_blue_rad[i]:
            t_i += 1
            t_ps.append(center_cyan) 
            t_ps_c.append('c')
      
        for center_magent in list_centers_Magenta:
          if self.get_distance_btwn_points(center_blue,center_magent) < list_blue_rad[i]:
            t_i += 1
            t_ps.append(center_magent) 
            t_ps_c.append('m')

        if t_i >= 3 :
          if self.debug_draws['tag_t_lines']:
            cv.line(frame,t_ps[0],t_ps[1],(255,0,0),2)
            cv.line(frame,t_ps[0],t_ps[2],(255,0,0),2)
            cv.line(frame,t_ps[1],t_ps[2],(107,237,170),3)
          
          x1,y1 = t_ps[0]
          x2,y2 = t_ps[1]
          x3,y3 = t_ps[2]

          c = ((x1+x2+x3)//3,(y1+y2+y3)//3)
          #d_m = get_distance_btwn_points(c,t_ps[0])
          d_a_deg = self.get_vert_inclination(t_ps[0],c)
          d_a = math.radians(d_a_deg)
          d_p = (c[0] - int(list_blue_rad[i]*0.8*math.sin(d_a)), c[1] - int(list_blue_rad[i]*0.8*math.cos(d_a)))
          #d_pp = (c[0] - int(d_m*math.sin(d_a +3.1416)), c[1] - int(d_m*math.cos(d_a+3.1416)))

          if self.debug_draws['tag_center']:
            cv.circle(frame, c, 5, (237,107,213), -1)
          if self.debug_draws['tag_perimeter']:
            cv.line(frame, c, d_p, (237,107,213), 2)
            #cv.line(frame, c, d_pp,(0,255,0), 2 )
            self.draw_rotated_rectangle(frame,c,list_yellow_rad[i]*1.7,d_a_deg)
          if self.debug_draws['angle_ref']:
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

          if self.debug_draws['tag_ID_text']:
            t = f'B-TAG-{self.tag_blue_ID[ID_found]}'
            cv.putText(frame, t, (c[0]-30,c[1]+20), cv.FONT_HERSHEY_SIMPLEX, 0.4,(0,0,0),2)

          t_ps.append(c)
          tag_main_b.append(t_ps)

          print(f"Se detecto un Tag!! \nCon centro en: {c}\nY una inclinazion resperto a la vertical de {round(d_a_deg)}°")
          print(f"El ID es '{ID_found}' del TAG {self.tag_blue_ID[ID_found]}")
      
    return frame

  def updateFrame(self):
    if self.webcam is not None:
      ret, self.current_frame = self.webcam.read()
      if ret:
        if self.enableHGP_videoTrasf:
          pts_origin = np.float32([self.HGP_point_values])
          pts_to_arrive = np.float32([(0,0),(self.camera_size[0],0),
                                    (0,self.camera_size[1]),self.camera_size])
          MTH = cv.getPerspectiveTransform(pts_origin,pts_to_arrive)
          self.current_frame = cv.warpPerspective(self.current_frame,MTH,self.camera_size)
        elif self.check_valid_HGPpoits():
          for i in range(len(self.HGP_point_values)):
            if self.HGP_point_values[i] is not None:
              cv.circle(self.current_frame, self.HGP_point_values[i], 4, (255, 0, 255), -1)
        
        if self.enableColorDetection:
          self.current_frame = self.doTagDetection(self.current_frame)
          """
          hsv = cv.cvtColor(self.current_frame, cv.COLOR_BGR2HSV)

          mask_yel = cv.inRange(hsv, np.array(self.Tcolors_LowRange['Yellow']), 
                                np.array(self.Tcolors_HighRange['Yellow']))
          mask_red = cv.inRange(hsv, np.array(self.Tcolors_LowRange['Red']), 
                                np.array(self.Tcolors_HighRange['Red']))
          
          # Encontrar contornos en la máscara
          #contours2, _ = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
          contours1, _ = cv.findContours(mask_yel, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

          contours = contours1
          for contour in contours:
            area = cv.contourArea(contour)
            if area > 100:  # Filtrar contornos pequeños
              # Calcular el rectángulo aproximado al contorno
              rect = self.encontrar_cuadrado_aprox(contour)
              # Calcular el centro del rectángulo
              centro = self.encontrar_centro(rect) 
              # Calcular el ángulo de rotación del contorno
              angulo = self.encontrar_angulo(contour)
              # Dibujar el cuadrado en la imagen
              box = cv.boxPoints(((rect[0] + rect[2] // 2, rect[1] + rect[3] // 2), (rect[2], rect[3]), 0))
              box = np.intp(box)
              cv.drawContours(self.current_frame, [box], 0, (0, 255, 0), 2)
              # Dibujar el centro del cuadrado en la imagen
              cv.circle(self.current_frame, centro, 5, (255, 0, 0), -1)
          """

    return self.current_frame

class StreamWebcamVideo(ttk.Frame):
  def __init__(self, parent, camera_info, config_tabs):
    #---Initial config
    super().__init__(parent)
    super().configure(relief= 'groove', border= 5)#añade un borde para ver los limites
    self.parent = parent

    #--- grid config
    self.rowconfigure(0,weight=1)
    self.columnconfigure(0,weight=1)

    #---Variables
    self.camera_info = camera_info
    self.config_tabs = config_tabs

    #---Widgets
    self.video_display = ttk.Label(
      self, justify='center', cursor='cross'#'plus'
    )

    #---Packing
    self.video_display.grid(
      row=0,column=0,sticky='nswe',
      padx=5, pady=5
    )
    #self.btn_toggle_video.grid(row=1,column=0,sticky='ns')

    #--- others
    self.updateFrame()
    self.video_display.bind('<Button-1>',self.on_mouse_click)

  def on_mouse_click(self,event):
    if self.camera_info.camera_status and self.config_tabs.index('current')==0:
      x, y = event.x -3, event.y-20
      x = max(min(x, self.camera_info.camera_size[0]), 0)
      y = max(min(y, self.camera_info.camera_size[1]), 0)
      
      self.camera_info.set_px_position(x, y)

  def updateFrame(self):
    if self.camera_info.webcam is not None:
        frame = self.camera_info.updateFrame()
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img = ImageTk.PhotoImage(
          image=img, width=self.camera_info.camera_size[0], height=self.camera_info.camera_size[1]
        )
        self.video_display.imgtk = img
        self.video_display.config(
          image=img
        )
    self.parent.after(10, self.updateFrame)

class CameraCalibration(ttk.Frame):
  def __init__(self, parent, camera_info):
    #---Initial config
    super().__init__(parent)
    super().configure(relief= 'groove', border= 5)#añade un borde para ver los limites
    self.parent = parent

    #--- grid config
    self.rowconfigure(0,weight=1)
    self.rowconfigure((1,2), weight=3)
    self.columnconfigure(1,weight=1)

    #---Variables
    self.camera_info = camera_info
    self.HGP_points_state = ( 
      tk.BooleanVar(value=False),tk.BooleanVar(value=False),
      tk.BooleanVar(value=False),tk.BooleanVar(value=False)
    )
    self.HGP_point_selected = 0

    #---Widgets
    self.CS_frame = ttk.Frame(
      self, relief='groove', border=5
    )
    self.CS_frame.columnconfigure(0, weight=1)
    self.CS_frame.rowconfigure((0,1,2), weight=1)
    self.Title_1 = tk.Label(
      self.CS_frame, text= "Camera Selector", font='Arial 10 bold',
      fg= 'black', justify='center'
    )
    self.camera_selector = ttk.Combobox(
      self.CS_frame, values = self.camera_info.available_cameras,
      state = 'readonly', justify='center', font= 'Arial 12 bold'
    )
    self.camera_selector.set(self.camera_info.available_cameras[0])
    self.btn_toggle_video = tk.Button(
      self.CS_frame, text = "Conect to webcam", command = self.toggle_video,
      fg = 'white', background = '#9fd9b3', font = 'Arial 12 bold'
    )


    self.LD_frame = ttk.Frame(
      self, relief='groove', border=5
    )
    self.LD_frame.columnconfigure(0, weight=1)
    self.LD_frame.rowconfigure((0,1,2), weight=1)

    self.Title_2 = tk.Label(
      self.LD_frame, text='Lens Distortion Calib.',
      fg='black'
    )
    self.btn_takePhoto = tk.Button(
      self.LD_frame, text='Take Photo', command=self.takePhoto,
      fg='white', background='#9fd9b3', font='Arial 12 bold'
    )
    self.btn_doLDcalib = tk.Button(
      self.LD_frame, text= 'LD Calib.', command=self.doLDcalib,
      fg='white', background='#9fd9b3', font='Arial 12 bold'
    )

    
    self.HGP_frame = ttk.Frame(
      self, relief='groove', border=5
    )
    self.HGP_frame.columnconfigure((0,1), weight=1)
    self.HGP_frame.rowconfigure((0,1,2,3,4,5,6), weight=1)

    self.Title_3 = tk.Label(
      self.HGP_frame, text='Homography Calib.',
      fg='black'
    )
    self.PtnSelec_1 = ttk.Checkbutton(
      self.HGP_frame, text= 'P1',
      onvalue=True, offvalue=False,
      variable= self.HGP_points_state[0],
      command= lambda: self.CheckButtons_eventHandler(0)##print(f'P1 state= {self.HGP_points_state[0].get()}')
    )
    self.PtnSelec_2 = ttk.Checkbutton(
      self.HGP_frame, text= 'P2',
      onvalue=True, offvalue=False,
      variable= self.HGP_points_state[1],
      command= lambda: self.CheckButtons_eventHandler(1)#print(f'P2 state= {self.HGP_points_state[1].get()}')
    )
    self.PtnSelec_3 = ttk.Checkbutton(
      self.HGP_frame, text= 'P3',
      onvalue=True, offvalue=False,
      variable= self.HGP_points_state[2],
      command= lambda: self.CheckButtons_eventHandler(2)#print(f'P3 state= {self.HGP_points_state[2].get()}')
    )
    self.PtnSelec_4 = ttk.Checkbutton(
      self.HGP_frame, text= 'P4',
      onvalue=True, offvalue=False,
      variable= self.HGP_points_state[3],
      command= lambda: self.CheckButtons_eventHandler(3)#print(f'P4 state= {self.HGP_points_state[3].get()}')
    )
    self.display_current_px = tk.Label(
      self.HGP_frame, background='white',
      fg='black', font='Arial 10', justify='center',
      text= 'HGP Current Point\n[px]'
    )
    self.btn_setHGP_point =  tk.Button(
      self.HGP_frame, text= 'SET POINT', command=self.setHGPpoint,
      fg='white', background='#9fd9b3', font='Arial 12 bold'
    )
    self.btn_resetHGPpoints = tk.Button(
      self.HGP_frame, text= 'RESET', command=self.resetHGPpointValues,
      fg='white', background='red', font='Arial 12 bold', state='disabled'
    )
    self.btn_doHomography = tk.Button(
      self.HGP_frame, text='Applied HGP', state='disabled',
      fg='white',font='Arial 12 bold', background='#9fd9b3',
      command=self.doHomography
    )

    #---Packing
    self.CS_frame.grid(
      row=0, column=0, sticky='nswe'
    )
    self.Title_1.grid(
      row=0,column=0,sticky='nswe'
    )
    self.camera_selector.grid(
      row=1, column=0, sticky='nswe',
      padx=10, pady=5
    )
    self.btn_toggle_video.grid(
      row=2, column=0, sticky='nswe',
      padx=10, pady=5
    )

    self.LD_frame.grid(
      row=1, column=0, sticky='nswe',
      pady=5
    )
    self.Title_2.grid(
      row=0, column=0, sticky='nswe'
    )
    self.btn_takePhoto.grid(
      row=1, column=0, sticky='nswe',
      padx=10, pady=5
    )
    self.btn_doLDcalib.grid(
      row=2, column=0, sticky='nswe',
      padx=10, pady=5
    )

    self.HGP_frame.grid(
      row=2, column=0, sticky='nswe',
      pady=5
    )
    self.Title_3.grid(
      row=0, column=0, columnspan=2,
      padx=10, pady=5
    )
    self.PtnSelec_1.grid(
      row=1, column=0
    )
    self.PtnSelec_2.grid(
      row=1, column=1
    )
    self.PtnSelec_3.grid(
      row=2, column=0
    )
    self.PtnSelec_4.grid(
      row=2, column=1
    )
    self.display_current_px.grid(
      row=3, column=0, columnspan=2,
      padx=10,pady=5, sticky='nswe'
    )
    self.btn_setHGP_point.grid(
      row=4, column=0, columnspan=2,
      padx=10, pady=5, sticky='nswe'
    )
    self.btn_resetHGPpoints.grid(
      row=5, column=0, columnspan=2,
      padx=10, pady=5, sticky='nswe'
    )
    self.btn_doHomography.grid(
      row=6, column=0, columnspan=2,
      padx=10, pady=5, sticky='nswe'
    )

    #--- others
    self.updateStatus()

  def CheckButtons_eventHandler(self,point):
    if self.HGP_points_state[point].get():
      self.HGP_point_selected = point+1
      for i in range(0,4):
        if i is not point:
          self.HGP_points_state[i].set(False)
    else:
      self.HGP_point_selected = 0
    #--- para Debug
    #print(f'Point selected = {self.HGP_point_selected}')

  def enable_LD_frame(self,enable):
    if enable:
      self.Title_2.config(state='normal')
      self.btn_doLDcalib.config(state='normal')
      self.btn_takePhoto.config(state='normal')
    else:
      self.Title_2.config(state='disabled')
      self.btn_doLDcalib.config(state='disabled')
      self.btn_takePhoto.config(state='disabled')

  def enable_HGP_frame(self,enable):
    if enable:
      self.Title_3.config(state='normal')
      self.btn_setHGP_point.config(state='normal')
      self.display_current_px.config(state='normal')
      self.PtnSelec_1.config(state='normal')
      self.PtnSelec_2.config(state='normal')
      self.PtnSelec_3.config(state='normal')
      self.PtnSelec_4.config(state='normal')
    else:
      self.Title_3.config(state='disabled')
      self.btn_setHGP_point.config(state='disabled')
      self.display_current_px.config(state='disabled')
      self.PtnSelec_1.config(state='disabled')
      self.PtnSelec_2.config(state='disabled')
      self.PtnSelec_3.config(state='disabled')
      self.PtnSelec_4.config(state='disabled')

  def toggle_video(self):
    if self.camera_info.webcam is None:
      # obtene el indice de la camara
      self.camera_info.setCameraIndex(int(self.camera_selector.get().split()[-1]))
      self.camera_info.connect_camera()
      if self.camera_info.camera_status:
        print(f'Dimenciones de imagen [px] = {self.camera_info.camera_size}')
    else:
      self.camera_info.disconnect_camera()

  def takePhoto(self):
    if self.camera_info.webcam is not None:
      ret, frame = self.camera_info.webcam.read()
      if ret:
        file_name = f'{path_image_folder}f{self.camera_info.photo_counter}_calib.png'
        cv.imwrite(file_name,frame)
        print(f"Foto tomada y guardada como '{file_name}'")
        self.camera_info.photo_counter += 1

  def setHGPpoint(self): 
    if self.HGP_point_selected != 0 and self.camera_info.current_px_position is not None:
      self.camera_info.HGP_point_values[self.HGP_point_selected-1] = self.camera_info.current_px_position
      if self.HGP_point_selected == 1:
        self.PtnSelec_1.config(text=f'P1:{self.camera_info.current_px_position}')
      elif self.HGP_point_selected == 2:
        self.PtnSelec_2.config(text=f'P2:{self.camera_info.current_px_position}')
      elif self.HGP_point_selected == 3:
        self.PtnSelec_3.config(text=f'P3:{self.camera_info.current_px_position}')
      elif self.HGP_point_selected == 4:
        self.PtnSelec_4.config(text=f'P4:{self.camera_info.current_px_position}')

      if self.btn_resetHGPpoints.cget('state') == 'disabled':
        self.btn_resetHGPpoints.config(state='normal')

      check = True
      for i in range(len(self.camera_info.HGP_point_values)):
        if self.camera_info.HGP_point_values[i] is None:
          check = False
      if check:
        self.btn_doHomography.config(state='normal')
      #--- para debug
      # print(self.camera_info.HGP_point_values)
    else:
      print('Primero seleciona un px y un punto para asignar!!')

  def resetHGPpointValues(self):
    self.camera_info.current_px_position = None
    self.HGP_point_selected = 0
    self.camera_info.enableHGP_videoTrasf = False
    for i in range(0,4):
      self.HGP_points_state[i].set(False)
      self.camera_info.HGP_point_values[i]= None
    
    self.display_current_px.config(text= 'HGP Current Point\n[px]')
    self.btn_resetHGPpoints.config(state='disabled')
    self.btn_doHomography.config(state='disabled')
    self.PtnSelec_1.config(text='P1')
    self.PtnSelec_2.config(text='P2')
    self.PtnSelec_3.config(text='P3')
    self.PtnSelec_4.config(text='P4')

    #--- para debug
    #print(self.camera_info.HGP_point_values)
    #print(self.camera_info.current_px_position)
    #print(self.HGP_point_selected)

  def doHomography(self):
    self.camera_info.enableHGP_videoTrasf = True

  def doLDcalib(self):
    pass

  def update_px_display(self):
    if self.camera_info.event_px_pos:
      x,y = self.camera_info.get_px_position()
      if x is not None and y is not None:
        self.display_current_px.config(text=f'HGP Current Point[px]:\n({x}, {y})')
      else:
        self.display_current_px.config(text='HGP Current Point[px]:\n(N/A)')

  def updateStatus(self):
    if self.camera_info.camera_status:
      self.btn_toggle_video.config(text = 'Stop Video',background = '#ff1d44')
      self.btn_takePhoto.config(state = 'normal')
      self.camera_selector.config(state = 'disabled')
      
      self.enable_LD_frame(True)
      self.enable_HGP_frame(True)

      self.update_px_display()
    else:
      self.btn_toggle_video.config(text = 'Conect to webcam', background = '#9fd9b3')
      self.btn_takePhoto.config(state = 'disabled')
      self.camera_selector.config(state = 'readonly')

      self.enable_LD_frame(False)
      self.enable_HGP_frame(False)
    
    self.parent.after(10, self.updateStatus)

class TagDetectionCalibration(ttk.Frame):
  def __init__(self, parent, camera_info , ):
    #--- Initial config
    super().__init__(parent)
    super().configure(relief= 'groove', border= 5)
    self.parent = parent

    #--- grid config
    self.columnconfigure(0, weight=1)
    self.rowconfigure((0,1),weight=1)

    #--- varibles
    self.camera_info = camera_info
    self.tag_colors_list = ('Yellow', 'Blue', 'Red', 'Cyan', 'Green','Magent', 'Orange')

    #--- widgets
    self.CC_frame = ttk.Frame(
      self, relief='groove', border=5
    )
    self.CC_frame.columnconfigure((0,1,2), weight=1)
    self.CC_frame.rowconfigure((0,1,2,3,4,5),weight=1)
    self.Title_1 = tk.Label(
      self.CC_frame, text='Color Calibration',
      justify='center'
    )
    self.color_selector = ttk.Combobox(
      self.CC_frame, values=self.tag_colors_list,
      state='readonly', justify='center'
    )
    self.color_selector.set(self.tag_colors_list[0])
    self.btn_doColorId = tk.Button(
      self.CC_frame, text='READ COLOR', command= self.doColorID,
      fg='white', background='#9fd9b3', state='disabled'
    )
    self.color_range_display = (
      tk.Label(     # LOW RANGE COLOR
        self.CC_frame,background='white',
        justify='center', text='Low Range'
      ),
      tk.Label(     # HIGH RANGE COLOR
        self.CC_frame, background='white',
        justify='center', text='High Range'
      ),
      tk.Label(     # AVERAGE COLOR
        self.CC_frame, background='grey',
        justify='center', text='Average Color',
        fg='white'
      )
    )
    self.current_color_range_display = (
      tk.Label(     # LOW RANGE COLOR
        self.CC_frame,background='white',
        justify='center', text='Low Range'
      ),
      tk.Label(     # HIGH RANGE COLOR
        self.CC_frame, background='white',
        justify='center', text='High Range'
      ),
      tk.Label(     # AVERAGE COLOR
        self.CC_frame, background='grey',
        justify='center', text='Average Color',
        fg='white'
      )
    )
    self.btns_set_range = (
      tk.Button(
        self.CC_frame, text='LOW', justify='center',
        background='#9fd9b3', fg='white', state='disabled',
        command=self.setLowCrange
      ),
      tk.Button(
        self.CC_frame, text='HIGH', justify='center',
        background='#9fd9b3', fg='white', state='disabled',
        command=self.setHighCrange
      )
    )
    self.btn_doDetectColors = tk.Button(
      self.CC_frame, text='ENABLE C.D.', command= self.enableColorDetect,
      fg='white', background='#9fd9b3', state='disabled'
    )

    #--- packing
    self.CC_frame.grid(
      row=0, column=0, sticky='nswe'
    )
    self.Title_1.grid(
      row=0,column=0, columnspan=3, sticky='nswe',
      padx=5, pady=5
    )
    self.current_color_range_display[0].grid(
      row=1, column=0, sticky='nswe',
      pady=5
    )
    self.current_color_range_display[2].grid(
      row=1, column=1, sticky='nswe',
      pady=5
    )
    self.current_color_range_display[1].grid(
      row=1, column=2, sticky='nswe',
      pady=5
    )
    self.color_selector.grid(
      row=2, column=0, columnspan=2, sticky='nswe',
      padx=5, pady=5
    )
    self.btn_doColorId.grid(
      row=2, column=2, sticky='nswe',
      padx=5, pady=5
    )
    self.color_range_display[0].grid(
      row=3, column=0, sticky='nswe',
      pady=5
    )
    self.color_range_display[2].grid(
      row=3, column=1, sticky='nswe',
      pady=5
    )
    self.color_range_display[1].grid(
      row=3, column=2, sticky='nswe',
      pady=5
    )
    self.btns_set_range[0].grid(
      row=4, column=0, sticky='nswe',
      padx=5, pady=5
    )
    self.btns_set_range[1].grid(
      row=4, column=2, sticky='nswe',
      padx=5, pady=5
    )
    self.btn_doDetectColors.grid(
      row=5, column=0, columnspan=3, sticky='nswe',
      padx=5, pady=5
    )

    #--- others
    self.updateStatus()
    self.updateColorDisplays()
    self.color_selector.bind("<<ComboboxSelected>>", lambda event: self.updateColorDisplays())

  def hsv_to_rgb(self,hsv):
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

  def doColorID(self):
    if self.camera_info.camera_status:
      frame = self.camera_info.current_frame
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

      self.camera_info.current_color_picked[0] = opaque_color.tolist()
      self.camera_info.current_color_picked[1] = shine_color.tolist()
      self.camera_info.current_color_picked[2] = prom_color.tolist()

      # Imprimir los resultados
      prom_color_rgb = self.hsv_to_rgb(prom_color)
      opaque_color_rgb = self.hsv_to_rgb(opaque_color)
      shine_color_rgb = self.hsv_to_rgb(shine_color)
      prom_color_hex = '#{:02x}{:02x}{:02x}'.format(prom_color_rgb[0], 
                                                    prom_color_rgb[1],
                                                    prom_color_rgb[2])
      opaque_color_hex = '#{:02x}{:02x}{:02x}'.format( opaque_color_rgb[0], 
                                                          opaque_color_rgb[1],
                                                          opaque_color_rgb[2])
      shine_color_hex = '#{:02x}{:02x}{:02x}'.format( shine_color_rgb[0], 
                                                              shine_color_rgb[1],
                                                              shine_color_rgb[2])
      self.color_range_display[2].config(
        background=prom_color_hex, text='Average Color:\n'+prom_color_hex
      )
      self.color_range_display[0].config(
        background=opaque_color_hex, text= 'LOW Range:\n'+opaque_color_hex
      )
      self.color_range_display[1].config(
        background=shine_color_hex, text= 'HIGH Range:\n'+shine_color_hex
      )
      
      #-debug
      #print("Color promedio (H, S, V):", prom_color)
      #print("Color más opaco (H, S, V):", opaque_color)
      #print("Color más brillante (H, S, V):", shine_color)

  def setLowCrange(self):
    if self.camera_info.check_valid_ColorPick():
      index = self.color_selector.get()
      self.camera_info.Tcolors_LowRange[index] = self.camera_info.current_color_picked[0]
      av = []
      av.append(int(np.mean([ self.camera_info.Tcolors_LowRange[index][0],
                          self.camera_info.Tcolors_HighRange[index][0]])))
      av.append(int(np.mean([ self.camera_info.Tcolors_LowRange[index][1],
                          self.camera_info.Tcolors_HighRange[index][1]])))
      av.append(int(np.mean([ self.camera_info.Tcolors_LowRange[index][2],
                          self.camera_info.Tcolors_HighRange[index][2]])))
      self.camera_info.Tcolors_Average[index] = av 

      self.camera_info.saveParameters()
      #-Debug
      #print(f'prom = {av}')
      #print(f'Save in Low range of {index}:{self.camera_info.current_color_picked[0]}')
    
    self.updateColorDisplays()
  
  def setHighCrange(self):
    if self.camera_info.check_valid_ColorPick():
      index = self.color_selector.get()
      self.camera_info.Tcolors_HighRange[index] = self.camera_info.current_color_picked[1]
      av = []
      av.append(int(np.mean([ self.camera_info.Tcolors_LowRange[index][0],
                          self.camera_info.Tcolors_HighRange[index][0]])))
      av.append(int(np.mean([ self.camera_info.Tcolors_LowRange[index][1],
                          self.camera_info.Tcolors_HighRange[index][1]])))
      av.append(int(np.mean([ self.camera_info.Tcolors_LowRange[index][2],
                          self.camera_info.Tcolors_HighRange[index][2]])))
      self.camera_info.Tcolors_Average[index] = av 
      self.camera_info.saveParameters()
      #-Debug
      #print(f'prom = {av}')
      #print(f'Save in High range of {index}:{self.camera_info.current_color_picked[1]}')
    self.updateColorDisplays()

  def enableColorDetect(self):
    self.camera_info.enableColorDetection = not self.camera_info.enableColorDetection
    print(f'Color Detection is {self.camera_info.enableColorDetection}')

  def enable_CC_frame(self,enable):
    if enable:
      self.Title_1.config(state='normal')
      self.color_selector.config(state='readonly')
      self.btn_doColorId.config(state='normal')
      self.color_range_display[2].config(state='normal')
      self.color_range_display[0].config(state='normal')
      self.color_range_display[1].config(state='normal')
      self.btns_set_range[0].config(state='normal')
      self.btns_set_range[1].config(state='normal')
      self.btn_doDetectColors.config(state='normal')
    else:
      self.Title_1.config(state='disabled')
      self.color_selector.config(state='disabled')
      self.btn_doColorId.config(state='disabled')
      self.color_range_display[2].config(state='disabled')
      self.color_range_display[0].config(state='disabled')
      self.color_range_display[1].config(state='disabled')
      self.btns_set_range[0].config(state='disabled')
      self.btns_set_range[1].config(state='disabled')
      self.btn_doDetectColors.config(state='disabled')

  def updateColorDisplays(self):
    # Imprimir los resultados
    index = self.color_selector.get()
    opaque_color = self.camera_info.Tcolors_LowRange[index]
    shine_color = self.camera_info.Tcolors_HighRange[index]
    average_color = self.camera_info.Tcolors_Average[index]

    average_color_rgb = self.hsv_to_rgb(average_color)
    opaque_color_rgb = self.hsv_to_rgb(opaque_color)
    shine_color_rgb = self.hsv_to_rgb(shine_color)
    prom_color_hex = '#{:02x}{:02x}{:02x}'.format(average_color_rgb[0], 
                                                  average_color_rgb[1],
                                                  average_color_rgb[2])
    opaque_color_hex = '#{:02x}{:02x}{:02x}'.format( opaque_color_rgb[0], 
                                                        opaque_color_rgb[1],
                                                        opaque_color_rgb[2])
    shine_color_hex = '#{:02x}{:02x}{:02x}'.format( shine_color_rgb[0], 
                                                            shine_color_rgb[1],
                                                            shine_color_rgb[2])
    self.current_color_range_display[2].config(
      background=prom_color_hex, text='Average Color:\n'+prom_color_hex
    )
    self.current_color_range_display[0].config(
      background=opaque_color_hex, text= 'LOW Range:\n'+opaque_color_hex
    )
    self.current_color_range_display[1].config(
      background=shine_color_hex, text= 'HIGH Range:\n'+shine_color_hex
    )
    pass

  def updateStatus(self):
    self.enable_CC_frame(self.camera_info.camera_status)
    #self.updateColorDisplays()
    self.parent.after(10,self.updateStatus)


#Variables para dimeciones de la HMI (Actualmente en desuso)
screen_w = 1000
screen_h = 600
size_frame = (int(0.9*screen_w),int(0.9*screen_h))

#---Definicion de la ventana principal de la HMI
window = tk.Tk() #window.geometry(f'{screen_w}x{screen_h}+0+0')

#---Config. grid
window.rowconfigure(0, weight=1)
window.columnconfigure(0, weight=1)
window.columnconfigure(1, weight=9)

#---main widgets
camera_struc = WebCamParameters()   #contenedor de parametros/variables para la camara
pestanas = ttk.Notebook(window)     #Pestañas para selecionar las configuraciones
video_frame = StreamWebcamVideo(window, camera_info=camera_struc, config_tabs=pestanas) #Display para el video
CalibCamara_frame = CameraCalibration(pestanas, camera_info=camera_struc)
CalibTags_frame = TagDetectionCalibration(pestanas,camera_info=camera_struc)#ttk.Frame(pestanas)    #proximanmete, config para detectar tags 

#---enpaquetado 
#Config. pestañas (añadiendo Frames para cada pestaña)
pestanas.add(CalibCamara_frame, text='CONFIG.')
pestanas.add(CalibTags_frame, text='TAG CALIB.')


pestanas.grid(row=0, column=0, sticky='nswe',padx=5)
video_frame.grid(row=0, column=1, sticky='nswe')

window.mainloop() #loop principal de la main app