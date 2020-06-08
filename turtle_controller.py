#!/usr/bin/env python3
"""turtle_controller."""

from controller import Robot
from controller import Supervisor
import numpy as np
import scipy.spatial.distance as distance
# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#DEF string have to be created for the robot as well as for the target
robot_node = robot.getFromDef("burger")
trans = robot_node.getField("translation")
rot = robot_node.getField ("rotation")

goal_node = robot.getFromDef("goal")
goal_trans = goal_node.getField('translation')

leftmotor = robot.getMotor ("left wheel motor")
rightmotor = robot.getMotor ("right wheel motor")

leftmotor.setPosition (float ("inf"))
rightmotor.setPosition (float ("inf"))

leftmotor.setVelocity (0)
rightmotor.setVelocity (0)

MAX_SPEED = 2.84

def boundingangle(x):
    while (x > np.pi):
        x -= 2 * np.pi
        
    while (x < -np.pi):
        x += 2 * np.pi
    return x

class pidcontroller:
    def __init__ (self, kp, ki, kd):
        self.kp = np.array(kp)
        self.ki = np.array(ki)
        self.kd = np.array(kd)
        self.cumm_error = np.array([0, 0])
        self.prev_error = np.array([0,0])
        
        
    def control(self, error, *agrs):
        error = np.array(error, dtype = float)
        self.cumm_error = np.add (self.cumm_error, error)
        dE = np.subtract(self.prev_error , error)
        self.prev_error = error
        return np.multiply(self.kp, error) + np.multiply(self.ki, self.cumm_error) + np.multiply(self.kd, dE)
        
#L and R are specific to robot kinematics
#vl and vr are specific for different drive model; this is for differential drive model
def burgermotordriver(v, w):
    L = 0.16
    R = 0.066
    def velocity(vel):
        return max(-1*MAX_SPEED, min(vel, MAX_SPEED))
    
    vr = (2.*v + w*L) / (2.*R)
    vl = (2.*v - w*L) / (2.*R)
    vr = velocity(vr)
    vl = velocity(vl)
    return [vr, vl]

pid2d = pidcontroller([2, 100], [0, 10], [0, 0])
    
heading_correction = -1.5708 + 3.14

robot_pos = np.array(trans.getSFVec3f()[0:3:2])
orientation = np.array(rot.getSFRotation()[3] + heading_correction)
robot_heading = boundingangle(orientation)



while robot.step(timestep) != -1:

    robot_pos = np.array(trans.getSFVec3f()[0:3:2])
    robot_pos[1] = -1 * robot_pos[1]
    
    
    orientation = np.array(rot.getSFRotation()[3] + heading_correction)
    robot_heading = orientation
    
    # goal_pos = [[0, +0.25], [0.25, 0.25]]
    
    goal_pos = np.array(goal_trans.getSFVec3f()[0:3:2])
    goal_pos[1] = -1 * goal_pos[1]
    
    #desired heading is angle towards the goal
    D_heading = np.arctan2(goal_pos[1] - robot_pos[1], goal_pos[0] - robot_pos[0])
    desired_heading = boundingangle( D_heading)
    
   
    
    reference = [goal_pos, desired_heading]
    state = [robot_pos, robot_heading]
    
    
    pos_error = distance.euclidean(goal_pos, robot_pos)
    angular_error = boundingangle( np.array (desired_heading - robot_heading))
    
     
    if (pos_error < 0.02):
        leftmotor.setVelocity (0)
        rightmotor.setVelocity (0)
        continue;       
    
    u = pid2d.control ([pos_error, angular_error]) 
    [vr, vl] = burgermotordriver (u[0], u[1]) 
    
    leftmotor.setVelocity (vl)
    rightmotor.setVelocity (vr)

       
    print( state, reference,np.degrees(robot_heading),pos_error, angular_error, [vr], [vl],u[0],u[1])

    pass
