import math

def get_m(A,B):
  x1,y1 = A
  x2,y2 = B
  if abs(x2 - x1) > 0:
    m = (y2-y1)/(x2-x1)
  else:
    m = 100000
  return m

def get_Horz_inclination(A,B):
  x1,y1 = A
  x2,y2 = B
  
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

def get_Vert_inclination(A,B):
  x1,y1 = A
  x2,y2 = B
  
  y_discr = (y2 - y1)
  x_discr = (x2 - x1)


  if(abs(x_discr) == 0): #poinst aliend vertically
    if y_discr >= 0 :
      theta_deg = 0
    else:
      theta_deg = 180
  elif(abs(y_discr) == 0): #poinst aliend horiz
    if x_discr >= 0 :
      theta_deg = 90
    else:
      theta_deg = -90
  else:
    m = (y2-y1)/(x2-x1)
    theta = math.atan(m)
    theta_deg = math.degrees(theta)

    if ((x_discr>0) and (y_discr>0)):
      theta_deg = -90 + theta_deg
    elif ((x_discr<0) and (y_discr<0)):
      theta_deg = 90 + theta_deg
    elif ((x_discr<0) and (y_discr>0)):
      theta_deg = 90 - theta_deg
    elif ((x_discr>0) and (y_discr<0)):
      theta_deg = -90 + theta_deg
  return theta_deg

A = (0,0)
B = (-1,-1)
m = get_m(A,B)
angle = math.degrees(math.atan(m))
c_ang = get_Horz_inclination(A,B)
v_ang = get_Vert_inclination(A,B)
print(f'Para A = {A} y B ={B}, -> m = {get_m(A,B)},\nangle = {angle} , angle_correct = {c_ang} \nang_vertical ={v_ang}')