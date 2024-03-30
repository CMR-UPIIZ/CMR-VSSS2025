import cv2 as cv
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import glob

path_image_folder = "Calib_images/"
path_camera_parameter = "../Camera_Cali_parameters"

class WebCamParameters:
  def __init__(self):
    self.available_cameras = self.get_available_cameras()
    self.selected_camera_index = 0
    self.webcam = None
    self.photo_counter = 1
    
  def get_available_cameras(self):
    camera_list = []
    for i in range(5):
      cam = cv.VideoCapture(i,cv.CAP_DSHOW)
      if cam.isOpened():
        camera_list.append(f'Camera {i}')
        cam.release()
    return camera_list
  
  def connect_camera(self, index):
    if index < len(self.available_cameras):
      self.selected_camera_index = index
      self.webcam = cv.VideoCapture(index, cv.CAP_DSHOW)
      if not self.webcam.isOpened():
        print(f"Error: No se pudo abrir la cámara {index}")
    else:
      print(f"Error: El índice de la cámara {index} está fuera de rango")

  def disconnect_camera(self):
    if self.webcam is not None:
      self.webcam.release()
      self.webcam = None

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
    self.rowconfigure((0,3), weight = 2)
    self.rowconfigure((1,2), weight = 1)

    #---Variables
    self.camera_parameters = WebCamParameters()
    #---Widgets
    self.camera_selector = ttk.Combobox(
      self, textvariable= self.camera_parameters.get_available_cameras,
      values = self.camera_parameters.available_cameras, state = 'readonly',
      justify='center', font= 'Arial 12 bold'
    )
    self.btn_toggle_video = tk.Button(
      self, text = "Conect to webcam", command = self.toggle_video,
      fg = 'white', background = '#9fd9b3', font = 'Arial 12 bold'
    )
    self.btn_takePhoto = tk.Button(
      self, text = 'Tomar Foto', command = self.takePhoto,
      fg = 'white', background = '#9fd9b3', font = 'Arial 12 bold'
    )
    self.video_display = ttk.Label(
      self
    )
    init_img =   tk.PhotoImage(file='Y_RG_code_2.png')
    self.video_display.imgtk = init_img
    self.video_display.config(image= init_img)
    
    #---Packing
    self.video_display.grid(
      row=0, rowspan=4, column=2, sticky='nswe'
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


  def toggle_video(self):
    if self.camera_parameters.webcam is None:
      # obtene el indice de la camara
      selected_camera_index = int(self.camera_selector.get().split()[-1])
      self.camera_parameters.connect_camera(selected_camera_index)
      if self.camera_parameters.webcam.isOpened():
        self.btn_toggle_video.config(text = 'Detener Video',background = '#ff1d44')
        self.btn_takePhoto.config(state = 'normal')
        self.camera_selector.config(state = 'disabled')
        image_size = (int(self.camera_parameters.webcam.get(cv.CAP_PROP_FRAME_WIDTH)),
                      int(self.camera_parameters.webcam.get(cv.CAP_PROP_FRAME_HEIGHT)))
        print(f'Dimenciones de imagen [px] = {image_size}')
    else:
      self.camera_parameters.disconnect_camera()
      self.btn_toggle_video.config(text = 'Conect to webcam', background = '#9fd9b3')
      self.btn_takePhoto.config(state = 'disabled')
      self.camera_selector.config(state = 'readonly')

  def takePhoto(self):
    if self.camera_parameters.webcam is not None:
      ret, frame = self.camera_parameters.webcam.read()
      if ret:
        file_name = f'{path_image_folder}foto_cali{self.camera_parameters.photo_counter}.png'
        cv.imwrite(file_name,frame)
        print(f"Foto tomada y guardada como '{file_name}'")
        self.camera_parameters.photo_counter += 1

  def updateFrame(self):
    if self.camera_parameters.webcam is not None:
      ret, frame = self.camera_parameters.webcam.read()
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

pestanas = ttk.Notebook(window, width=  int(0.9*screen_w), height= int(1*screen_h))

ColorCalib_frame = ColorCalibrationApp(pestanas,size=(int(0.9*screen_w),int(0.9*screen_h)))
void_frame = ttk.Frame(pestanas)

pestanas.add(ColorCalib_frame, text='CONFIGURACION')
pestanas.add(void_frame, text='pestaña 2')

pestanas.pack(expand= True,fill='both')

window.mainloop()