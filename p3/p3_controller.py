import random
from enum import Enum
import numpy as np

from controller import Robot, Motor, DistanceSensor

TIME_STEP = 32
MAX_SPEED = 10

gamma = 0.5
learning_rate = 0.5
Q = np.zeros((3, 3))
vis = np.zeros((3, 3))
last_second = 0

current_state = 2
current_action = 0

ultrasonic_sensors = [
    "left ultrasonic sensor",
    "front left ultrasonic sensor",
    "front ultrasonic sensor",
    "front right ultrasonic sensor",
    "right ultrasonic sensor"
]

infrared_sensors = [
    "rear left infrared sensor", 
    "left infrared sensor", 
    "front left infrared sensor", 
    "front infrared sensor",
    "front right infrared sensor", 
    "right infrared sensor", 
    "rear right infrared sensor", 
    "rear infrared sensor",
    "ground left infrared sensor", 
    "ground front left infrared sensor", 
    "ground front right infrared sensor",
    "ground right infrared sensor"
]


def init_devices():

    robot = Robot()
       
    leds = [robot.getDevice("front left led"), robot.getDevice("front right led"), robot.getDevice("rear led")]
    
    leftWheel = robot.getDevice("left wheel motor")
    rightWheel = robot.getDevice("right wheel motor")
    leftWheel.getPositionSensor().enable(TIME_STEP)
    rightWheel.getPositionSensor().enable(TIME_STEP)
    leftWheel.setPosition(float('inf'))
    rightWheel.setPosition(float('inf'))
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    
    ultrasonic_sensors_init = []
    for sensor in ultrasonic_sensors:
        ultr_sens = robot.getDevice(sensor)
        ultr_sens.enable(TIME_STEP)
        ultrasonic_sensors_init.append(ultr_sens)
    
    infrared_sensors_init = []
    for sensor in infrared_sensors:
        infr_sens = robot.getDevice(sensor)
        infr_sens.enable(TIME_STEP)
        infrared_sensors_init .append(infr_sens)
    
    return robot, leds, leftWheel, rightWheel, ultrasonic_sensors_init, infrared_sensors_init

def check_sensors():
    return [infrared_sensors[8].getValue(), infrared_sensors[9].getValue(),
            infrared_sensors[10].getValue(), infrared_sensors[11].getValue()]
              
def get_state(infrared_sensors):
    if sensor_values[1] > 750 and sensor_values[3] < 500:
        return 0
    elif sensor_values[2] > 750 and sensor_values[0] < 500:
        return 1
    else:
        return 2
    
def go_forward():
    action = 0
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
def turn_right():
    action = 1
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(-MAX_SPEED)
    
def turn_left():
    action = 2
    leftWheel.setVelocity(-MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
def do_action(action):
    if action == 0:
        turn_right()
    elif action == 1:
        turn_left()
    elif action == 2:
        go_straight()
    
def update_refuerzo(refuerzo, action, prev_state, next_state, learning_rate):
    vis[prev_estado][action] += 1
    learning_rate = 1 / (1 + visitas[prev_estado][action])
    mat_q[prev_estado][action] = (1-learning_rate) * mat_q[prev_estado][action] + learning_rate * (refuerzo + 0.5 * np.argmax(mat_q[nuevo_estado]))
    return learning_rate
    
def check_refuerzo(new_values, prev_values):
    if all(value < 500 for value in prev_sensor_values) and all(value < 500 for value in new_sensor_values):
        return 1
    elif all(value < 500 for value in prev_sensor_values) and not all(value < 500 for value in new_sensor_values):
        return -1
    elif all(value > 750 for value in prev_sensor_values) and all(value > 750 for value in new_sensor_values):
        return -1
    elif all(value > 750 for value in prev_sensor_values) and not all(value > 750 for value in new_sensor_values):
        return 1
    elif sum(i > 750 for i in prev_sensor_values) < sum(i > 750 for i in new_sensor_values):
        return -1
    else:
        return 1
    
def select_action(estado_actual):
    return np.argmax(mat_q[estado_actual])   
    
def main():
    global last_second
    robot, leds, leftWheel, rightWheel, ultrasonic_sensors, infrared_sensors = init_devices()
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
    while robot.step(TIME_STEP) != -1:
        current_second = robot.getTime()
        if current_second != last_second:
            last_second = current_second
            
    
if __name__ == "__main__":
    main()
 

    
    
    
    
    
    