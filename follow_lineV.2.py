from GUI import GUI
from HAL import HAL
import time
import math
import cv2
import numpy as np

#variables
current_time = 0
pre_time = 0
pre_err = 0
V_max = 5

def image_processing(image):
  
  white_pixels_straight = []
  white_pixels_curve = []
  pixel_count_straight = 0
  pixel_count_curve = 0
  err_straight = 0
  err_curve = 0
  
    
  lower = np.array([15, 15, 100], dtype = "uint8")    
  upper = np.array([50, 50, 255], dtype = "uint8")
    
  mask = cv2.inRange(image, lower, upper)

  output = cv2.bitwise_and(image, image, mask=mask)
  output[mask == 255] = [255, 255, 255]
  
  for i in range(0,640):
    if(output[250][i] == [255, 255, 255]).all():
      pixel_count_straight += 1
      output[250][i] = [255, 0, 255]
      white_pixels_straight.append(i)
        
  position_straight = pixel_count_straight//2
  if pixel_count_straight != 0:
    err_straight = 320 - white_pixels_straight[position_straight]

  for i in range(0,640):
    if(output[245][i] == [255, 255, 255]).all():
      pixel_count_curve += 1
      output[245][i] = [255, 255, 0]
      white_pixels_curve.append(i)
        
  position_curve = pixel_count_curve//2
  if pixel_count_curve != 0:
      err_curve = 320 - white_pixels_curve[position_curve]
      print(err_curve)

  return err_straight, err_curve, output
  
def control_PID(err_straight, err_curve, pre_err):
  current_time = float(time.time())
  sum_error_straight = 0
    
  d_error_straight = float(err_straight) - float(pre_err)
  d_time = current_time - pre_time
    
  sum_error_straight += sum_error_straight * (d_time)

  if -15 < err_curve < 15:
    print("recta")
    v = 10
      
    kp_w = 0.009
    kd_w = 0.0008
    ki_w = 0.008
      
  else:
    print("curva")
    v = 6
      
    kp_w = 0.0089
    kd_w = 0.0035
    ki_w = 0.008
      
  p_w = kp_w * err_straight
  d_w = kd_w * (d_error_straight / d_time)
  i_w = ki_w * sum_error_straight
    
  if d_w > 15:
    d_w = 15
        
  if d_w < -15:
    d_w = -15
				
  u_w = p_w + d_w + i_w
    
  return v, u_w
  
while True:
  
    image = HAL.getImage()

    err_straight, err_curve, output = image_processing(image)
    
    v, u_w = control_PID(err_straight, err_curve, pre_err)
    
    GUI.showImage(output)

    HAL.setV(v)
    HAL.setW(u_w)
    
    pre_time = float(time.time())
    pre_err = float(err_straight)
