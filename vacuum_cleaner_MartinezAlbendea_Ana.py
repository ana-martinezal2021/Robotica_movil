from GUI import GUI
from HAL import HAL
import random
import rospy
import time
import math

#Variables
state, v, w = 0, 0, 2
impediment = time.time()

#Function
def parse_laser_data(laser_data):
    laser = []
    for i in range(180):
        dist = laser_data.values[i]
        angle = math.radians(i)
        laser += [(dist, angle)]
    return laser
    
while True:

		#States    
    if state == 0:
      
      HAL.setV(v)
      HAL.setW(w)
      
      v += 0.0055
      
      laser_data = HAL.getLaserData()
      info = parse_laser_data(laser_data)
      
      for i in range(40):
        if info[70 + i][0] < 0.3:
          impediment = time.time()
          state = 1
          
    elif state == 1:
      
      HAL.setV(-1.5)
      
      if (time.time() - impediment) > 1:
        impediment = time.time()
        HAL.setV(0)
        turn_speed = random.uniform(-3,3)
        state = 2
        
    elif state == 2:
      
      HAL.setW(turn_speed)
      
      if (time.time() - impediment) > 2:
        impediment = time.time()
        HAL.setW(0)
        state = 3
        
    elif state == 3:
      
      HAL.setV(2)
      
      laser_data = HAL.getLaserData()
      info = parse_laser_data(laser_data)
      
      for i in range(40):
          if info[70 + i][0] < 0.3:
            impediment = time.time()
            state = 1
      
      if ((time.time() - impediment) > random.uniform(3,6)) and (info[90][0] > 1.5):
        v = 0
        w = 2
        state = 0
