import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

path_image_folder = "Calib_images/"

class WebcamApp(ttk.Frame):
	def __init__(self, parent):
		super().__init__(parent)
		#self.config(width=size[0],height=size[1])
		self.parent = parent
		self.parent.title("Aplicación de Webcam")

		self.available_cameras = self.get_available_cameras()
		self.selected_camera = tk.StringVar()
		self.selected_camera.set(self.available_cameras[0])

		self.camera_selector = ttk.Combobox(
			parent,textvariable = self.selected_camera, values= self.available_cameras
		)
		self.camera_selector.pack(anchor='center')

		self.cap = None
		self.photo_counter = 1  # Inicializa el contador de fotos

		self.label = ttk.Label(parent)
		self.label.pack(expand=True)

		self.btn_toggle_video = tk.Button(
			parent, text="Iniciar Video", command=self.toggle_video, fg= '#ffffff',
			background = '#9fd9b3', font= 'Arial 22 bold'
		)
		self.btn_toggle_video.pack(expand=True,fill='both')

		self.btn_capture = tk.Button(
			parent, text="Tomar Foto", command=self.capture_image, state="disabled",
			fg= '#ffffff', background='#3ea3af', font= 'Arial 22 bold'
		)
		self.btn_capture.pack(expand=True,fill='both')

		self.update_frame()

	def update_frame(self):
		if self.cap is not None:
			ret, frame = self.cap.read()
			if ret:
				frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
				img = Image.fromarray(frame_rgb)
				img = ImageTk.PhotoImage(image=img)
				self.label.imgtk = img
				self.label.config(image=img)
		self.parent.after(10, self.update_frame)

	def toggle_video(self):
		if self.cap is None:
			selected_camera_index = int(self.selected_camera.get().split()[-1])  # Obtener el índice de la cámara seleccionada
			self.cap = cv2.VideoCapture(selected_camera_index, cv2.CAP_DSHOW)
			if self.cap.isOpened():
				self.btn_toggle_video.config(text="Detener Video", background= '#ff1d44')
				self.btn_capture.config(state='normal')
				self.camera_selector.config(state='disabled')
		else:
			self.cap.release()
			self.cap = None
			self.btn_toggle_video.config(text="Iniciar Video", background= '#9fd9b3')
			self.btn_capture.config(state='disabled')
			self.camera_selector.config(state='normal')

	def capture_image(self):
		if self.cap is not None:
			ret, frame = self.cap.read()
			if ret:
				file_name = f"{path_image_folder}foto{self.photo_counter}.png"  # Nombre del archivo con contador
				cv2.imwrite(file_name, frame)
				print(f"Foto tomada y guardada como '{file_name}'")
				self.photo_counter += 1  # Incrementa el contador de fotos

	def get_available_cameras(self):
		cameras = []
		for i in range(10):
			cap = cv2.VideoCapture(i,cv2.CAP_DSHOW)
			if cap.isOpened():
				cameras.append(f'Camara {i}')
				cap.release()
		return cameras

#if __name__ == "__main__":
screen_w = 750
screen_h = 650

window = tk.Tk()
window.geometry(f'{screen_w}x{screen_h}+0+0')

app = WebcamApp(window) #frame para la app de captura de fotos 
app.pack(expand=True)

window.mainloop()
