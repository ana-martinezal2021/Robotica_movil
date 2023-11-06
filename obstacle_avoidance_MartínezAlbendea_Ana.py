from GUI import GUI
from HAL import HAL
import math
import numpy as np

def laser_vector(laser):
    laser_vectorized = []
    for d,a in laser:
        # (4.2.1) laser into GUI reference system
        x = 1/d * math.cos(a) * -1
        y = 1/d * math.sin(a) * -1
        v = (x,y)
        laser_vectorized += [v]

    laser_mean = np.mean(laser_vectorized, axis=0)
    return laser_mean

def absolute2relative (x_abs, y_abs, robotx, roboty, robott):

    # robotx, roboty are the absolute coordinates of the robot
    # robott is its absolute orientation
    # Convert to relatives
    dx = x_abs - robotx
    dy = y_abs - roboty

    # Rotate with current angle
    x_rel = dx * math.cos (-robott) - dy * math.sin (-robott)
    y_rel = dx * math.sin (-robott) + dy * math.cos (-robott)

    return x_rel,y_rel


def parse_laser_data (laser_data):
    laser = []
    for i,dist in enumerate(laser_data.values):
        if dist > 10:
            dist = 10
        angle = math.radians(i-90) # because the front of the robot is -90 degrees
        laser += [(dist, angle)]
    return laser

while True:
    
    image = HAL.getImage()
    GUI.showImage(image)
    
    currentTarget = GUI.map.getNextTarget()
    
    robot_x = HAL.getPose3d().x
    robot_y = HAL.getPose3d().y
    robot_or = HAL.getPose3d().yaw
    
    target_abs_x = currentTarget.getPose().x
    target_abs_y = currentTarget.getPose().y
    
    local_target = absolute2relative(target_abs_x, target_abs_y, robot_x, robot_y, robot_or)

    laser = HAL.getLaserData()
    laser_info = parse_laser_data(laser)
    obst_coor = laser_vector(laser_info)
    
    obs_x = obst_coor[0]
    obs_y = obst_coor[1]

    local_x = local_target[0]
    local_y = local_target[1]
    
    tg_distance = math.sqrt((local_x**2) + (local_y**2))

		# Restricting the distance to reach the target
    if tg_distance < 2.5:
      currentTarget.setReached(True)   

    GUI.showLocalTarget(local_target)

		# Restricting the local target vector
    if local_target[0] > 4.7:
      local_x = 4.7
      
    if local_target[1] > 4.7:
      local_y = 4.7
    
    # Car direction  (green line in the image below)
    carForce = [0.75*local_x, 0.25*local_y]
    # Obstacles direction (red line in the image below)
    obsForce = [11*obs_x, 11*obs_y]
    # Average direction (black line in the image below)
    avgForce = [2*carForce[0] + 0.75*obsForce[0], 2*carForce[1] + 0.75*obsForce[1]]
      
    # Setting the average force if the distance to the target is large
    if tg_distance > 30:
      avgForce = [4, 1.75*obsForce[1]]      
  
    GUI.showForces(carForce, obsForce, avgForce)

    HAL.setV(avgForce[0])
    HAL.setW(avgForce[1])

    
    
    
    
