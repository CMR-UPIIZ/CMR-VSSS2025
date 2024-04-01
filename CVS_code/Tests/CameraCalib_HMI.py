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

class CameraCalibrationPanel(ttk.Frame):
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
    self.HGP_frame.rowconfigure((0,1,2,3,4,5), weight=1)

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

    #--- others
    self.updateStatus()

  def CheckButtons_eventHandler(self,point):
    if self.HGP_points_state[point].get():
      self.HGP_point_selected = point+1
      for i in range(4):
        if i is not point:
          self.HGP_points_state[i].set(False)
    else:
      self.HGP_point_selected = 0
    print(f'Point selected = {self.HGP_point_selected}')

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
        file_name = f'{path_image_folder}f{self.camera_info.photo_counter}_calib.png'
        cv.imwrite(file_name,frame)
        print(f"Foto tomada y guardada como '{file_name}'")
        self.camera_info.photo_counter += 1

  def setHGPpoint(self): 
    print('AÑADE la funcionalidad')
    #La idea es que que check que boton esta selecionado
    #mediante la variable self.HGP_point_selected y si
    #existe un pixel selecionado, asigigne la posicion del px
    #al punto respectivo para la homografia
    

  def resetHGPpointValues(self):
    pass

  def doLDcalib(self):
    pass

  def updateStatus(self):
    if self.camera_info.camera_status:
      self.btn_toggle_video.config(text = 'Stop Video',background = '#ff1d44')
      self.btn_takePhoto.config(state = 'normal')
      self.camera_selector.config(state = 'disabled')
      
      self.enable_LD_frame(True)
      self.enable_HGP_frame(True)
    else:
      self.btn_toggle_video.config(text = 'Conect to webcam', background = '#9fd9b3')
      self.btn_takePhoto.config(state = 'disabled')
      self.camera_selector.config(state = 'readonly')

      self.enable_LD_frame(False)
      self.enable_HGP_frame(False)
    
    self.parent.after(10, self.updateStatus)

class StreamWebcamVideo(ttk.Frame):
  def __init__(self, parent, camera_info):
    #---Initial config
    super().__init__(parent)
    super().configure(relief= 'groove', border= 5)#añade un borde para ver los limites
    self.parent = parent

    #--- grid config
    self.rowconfigure(0,weight=1)
    self.columnconfigure(0,weight=1)

    #---Variables
    self.camera_info = camera_info

    #---Widgets
    self.video_display = ttk.Label(
      self, justify='center'
    )

    #---Packing
    self.video_display.grid(row=0,column=0,sticky='nswe')
    #self.btn_toggle_video.grid(row=1,column=0,sticky='ns')

    #--- others
    self.updateFrame()

  def updateFrame(self):
    if self.camera_info.webcam is not None:
      ret, frame = self.camera_info.webcam.read()
      if ret:
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        img = Image.fromarray(frame_rgb)
        img = ImageTk.PhotoImage(image=img)
        self.video_display.imgtk = img
        self.video_display.config(image=img)
    self.parent.after(10, self.updateFrame)

#Variables para dimeciones de la HMI (Actualmente en desuso)
screen_w = 1000
screen_h = 600
size_frame = (int(0.9*screen_w),int(0.9*screen_h))

#---Definicion de la ventana principal de la HMI
window = tk.Tk() #window.geometry(f'{screen_w}x{screen_h}+0+0')

#---Config. grid
window.rowconfigure(0, weight=1)
window.columnconfigure(0, weight=1)
window.columnconfigure(1,weight=9)

#---main widgets
camera_struc = WebCamParameters()   #contenedor de parametros/variables para la camara
video_frame = StreamWebcamVideo(window, camera_info=camera_struc) #Display para el video
pestanas = ttk.Notebook(window)     #Pestañas para selecionar las configuraciones
Calib_frame = CameraCalibrationPanel(pestanas, camera_info=camera_struc)
void_frame = ttk.Frame(pestanas)    #proximanmete, config para detectar tags 

#---enpaquetado 
#Config. pestañas (añadiendo Frames para cada pestaña)
pestanas.add(Calib_frame, text='CONFIG.')
pestanas.add(void_frame, text='TAG CALIB.')


pestanas.grid(row=0, column=0, sticky='nswe')
video_frame.grid(row=0, column=1, sticky='nswe')

window.mainloop() #loop principal de la main app