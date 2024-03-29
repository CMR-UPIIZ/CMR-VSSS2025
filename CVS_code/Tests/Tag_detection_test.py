import cv2 as cv
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

path_image_folder = "Calib_images/"

class ColorCalibrationApp(ttk.Frame):
  def __init__(self, parent, size):
    #---Initial config
    super().__init__(parent)
    super().configure(relief= 'groove', border= 5)#añade un borde para ver los limites
    self.parent = parent
    #self.parent.title('Calibracion de Color')

    #--- grid config
    self.config(width= size[0], height=size[1])
    self.columnconfigure((0,1), weight = 1)
    self.columnconfigure(2, weight = 9)
    self.rowconfigure((0,3), weight = 1)
    self.rowconfigure((1,2), weight = 2)

    #---Variables
    self.available_cameras = self.get_available_cameras()
    self.selected_camera = tk.StringVar() #variable para seleccion de camara
    self.selected_camera.set(self.available_cameras[0]) #primera opcion default
    self.webcam = None
    self.photo_counter = 1

    #---Widgets
    self.camera_selector = ttk.Combobox(
      parent, textvariable= self.selected_camera,
      values = self.available_cameras, state = 'normal' 
    )
    self.btn_toggle_video = tk.Button(
      parent, text = "Conect to webcam", command = self.toggle_video,
      fg = 'white', background = '#9fd9b3', font = 'Arial 18 bold'
    )
    self.btn_takePhoto = tk.Button(
      parent, text = 'Tomar Foto', command = self.takePhoto,
      fg = 'white', background = '#9fd9b3', font = 'Arial 18 bold'
    )
    self.video_display = ttk.Label(
      parent
    )
    
    #---Packing
    self.video_display.grid(
      row=0, rowspan=4, column=2, sticky='we'
    )
    self.camera_selector.grid(
      row=1, column=0, sticky='nswe'
    )
    self.btn_toggle_video.grid(
      row=1, column=1, sticky='nswe'
    )
    self.btn_takePhoto.grid(
      row=2, column=0, columnspan=2, sticky='nswe'
    )

    #--- others
    self.updateFrame()

  def get_available_cameras(self):
    camera_list = []
    for i in range(10):
      cam = cv.VideoCapture(i,cv.CAP_DSHOW)
      if cam.isOpened():
        camera_list.append(f'Camera {i}')
        cam.release()
    return camera_list

  def toggle_video(self):
    if self.webcam is None:
      # obtene el indice de la camara
      selected_camera_index = int(self.selected_camera.get().split()[-1])
      self.webcam = cv.VideoCapture(selected_camera_index, cv.CAP_DSHOW)
      if self.webcam.isOpened():
        self.btn_toggle_video.config(text = 'Detener Video',background = '#ff1d44')
        self.btn_takePhoto.config(state = 'normal')
        self.camera_selector.config(state = 'disabled')
    else:
      self.webcam.release()
      self.webcam = None
      self.btn_toggle_video.config(text = 'Conect to webcam', background = '#9fd9b3')
      self.btn_takePhoto.config(state = 'disabled')
      self.camera_selector.config(state = 'normal')

  def takePhoto(self):
    if self.webcam is not None:
      ret, frame = self.webcam.read()
      if ret:
        file_name = f'{path_image_folder}foto_cali{self.photo_counter}'
        cv.imwrite(file_name,frame)
        print(f"Foto tomada y guardada como '{file_name}'")
        self.photo_counter += 1

  def updateFrame(self):
    if self.webcam is not None:
      ret, frame = self.webcam.read()
      if ret:
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img = ImageTk.PhotoImage(image=img)
        self.video_display.imgtk = img
        self.video_display.config(image=img)
    self.parent.after(10, self.updateFrame)

screen_w = 1000
screen_h = 600

window = tk.Tk()
window.geometry(f'{screen_w}x{screen_h}+0+0')

pestanas = ttk.Notebook(window, width=  int(0.9*screen_w), height= int(0.9*screen_h))

ColorCalib_frame = ColorCalibrationApp(pestanas,size=(int(0.9*screen_w),int(0.9*screen_h)))
void_frame = ttk.Frame(pestanas)

pestanas.add(ColorCalib_frame, text='pestaña 1')
pestanas.add(void_frame, text='pestaña 2')

pestanas.pack(expand= True)

window.mainloop()









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
