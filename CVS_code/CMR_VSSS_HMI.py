import cv2 as cv
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import json
import numpy as np

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
    self.tag_colors_list = ('Yellow', 'Blue', 'Red', 'Cyan', 'Green','Magent')

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