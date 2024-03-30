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
    self.selected_camera_index = None
    self.webcam = None
    self.photo_counter = 1
    self.camera_size = ()
    self.camera_status = False
    
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

class CameraCalibrationApp(ttk.Frame):
  def __init__(self, parent, size, camera_info):
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
    self.camera_info = camera_info
    #---Widgets
    self.camera_selector = ttk.Combobox(
      self,values = self.camera_info.available_cameras,
      state = 'readonly',justify='center', font= 'Arial 12 bold'
    )
    self.camera_selector.set(self.camera_info.available_cameras[0])
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
    if self.camera_info.webcam is None:
      # obtene el indice de la camara
      self.camera_info.setCameraIndex(int(self.camera_selector.get().split()[-1]))
      self.camera_info.connect_camera()
      if self.camera_info.webcam.isOpened():
        
        image_size = (int(self.camera_info.webcam.get(cv.CAP_PROP_FRAME_WIDTH)),
                      int(self.camera_info.webcam.get(cv.CAP_PROP_FRAME_HEIGHT)))
        print(f'Dimenciones de imagen [px] = {image_size}')
    else:
      self.camera_info.disconnect_camera()

  def takePhoto(self):
    if self.camera_info.webcam is not None:
      ret, frame = self.camera_info.webcam.read()
      if ret:
        file_name = f'{path_image_folder}foto_cali{self.camera_info.photo_counter}.png'
        cv.imwrite(file_name,frame)
        print(f"Foto tomada y guardada como '{file_name}'")
        self.camera_info.photo_counter += 1

  def updateFrame(self):
    if self.camera_info.webcam is not None and self.parent.index('current')==0:
      ret, frame = self.camera_info.webcam.read()
      if ret:
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img = ImageTk.PhotoImage(image=img)
        self.video_display.imgtk = img
        self.video_display.config(image=img)
    else:
      pass
    if self.camera_info.camera_status:
      self.btn_toggle_video.config(text = 'Detener Video',background = '#ff1d44')
      self.btn_takePhoto.config(state = 'normal')
      self.camera_selector.config(state = 'disabled')
    else:
      self.btn_toggle_video.config(text = 'Conect to webcam', background = '#9fd9b3')
      self.btn_takePhoto.config(state = 'disabled')
      self.camera_selector.config(state = 'readonly')
    
    self.parent.after(10, self.updateFrame)

class StreamWebcamVideo(ttk.Frame):
  def __init__(self, parent, size , camera_info):
    #---Initial config
    super().__init__(parent)
    super().configure(relief= 'groove', border= 5)#añade un borde para ver los limites
    self.parent = parent

    #--- grid config
    self.config(width= size[0], height=size[1])
    self.rowconfigure(0,weight=5)
    self.rowconfigure(1, weight=1)
    self.columnconfigure(0,weight=1)

    #---Variables
    self.camera_info = camera_info

    #---Widgets
    self.video_display = ttk.Label(
      self
    )
    self.btn_toggle_video = tk.Button(
      self, text="STAR VIDEO", command= self.toggle_video,
      background='green', fg='white', state='normal'
    )

    #---Packing
    self.video_display.grid(row=0,column=0,sticky='nswe')
    self.btn_toggle_video.grid(row=1,column=0,sticky='ns')

    #--- others
    self.updateFrame()

  def toggle_video(self):
    if self.camera_info.webcam is None:
      print(self.camera_info.selected_camera_index)
      self.camera_info.connect_camera()
    else:
      self.camera_info.disconnect_camera()
  
  def updateFrame(self):
    if self.camera_info.webcam is not None and self.parent.index('current')==1:
      ret, frame = self.camera_info.webcam.read()
      if ret:
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img = ImageTk.PhotoImage(image=img)
        self.video_display.imgtk = img
        self.video_display.config(image=img)
    else:
      pass
    
    if self.camera_info.camera_status:
      self.btn_toggle_video.config(text='STOP VIDEO', background='red')
    else:
      self.btn_toggle_video.config(text='START VIDEO', background='green')
    self.parent.after(10, self.updateFrame)


screen_w = 1000
screen_h = 600

window = tk.Tk()
window.geometry(f'{screen_w}x{screen_h}+0+0')

pestanas = ttk.Notebook(window, width=int(0.9*screen_w), height=int(1*screen_h))

size_frame = (int(0.9*screen_w),int(0.9*screen_h))

camera_struc = WebCamParameters()
Calib_frame = CameraCalibrationApp(pestanas, size=size_frame, camera_info=camera_struc)
video_frame = StreamWebcamVideo(pestanas, size=size_frame, camera_info= camera_struc)

pestanas.add(Calib_frame, text='CONFIGURACION')
pestanas.add(video_frame, text='pestaña 2')

pestanas.pack(expand= True,fill='both')

window.mainloop()