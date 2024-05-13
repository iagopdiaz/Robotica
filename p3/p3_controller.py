import numpy as np
from controller import Robot, Motor, DistanceSensor


TIME_STEP = 32
MAX_SPEED = 10
GAMMA = 0.5  
LEARNING_RATE = 0.5
STATE_COUNT = 3
ACTION_COUNT = 3

Q = np.zeros((STATE_COUNT, ACTION_COUNT))
visits = np.zeros((STATE_COUNT, ACTION_COUNT))
current_state = 2
last_second = 0

ULTRASONIC_SENSORS = [
    "left ultrasonic sensor", "front left ultrasonic sensor",
    "front ultrasonic sensor", "front right ultrasonic sensor",
    "right ultrasonic sensor"
]
INFRARED_SENSORS = [
    "rear left infrared sensor", "left infrared sensor",
    "front left infrared sensor", "front infrared sensor",
    "front right infrared sensor", "right infrared sensor",
    "rear right infrared sensor", "rear infrared sensor",
    "ground left infrared sensor", "ground front left infrared sensor",
    "ground front right infrared sensor", "ground right infrared sensor"
]

def init_devices():

    robot = Robot()
    
    left_wheel = robot.getDevice("left wheel motor")
    right_wheel = robot.getDevice("right wheel motor")
    left_wheel.getPositionSensor().enable(TIME_STEP)
    right_wheel.getPositionSensor().enable(TIME_STEP)
    left_wheel.setPosition(float('inf'))
    right_wheel.setPosition(float('inf'))
    left_wheel.setVelocity(0)
    right_wheel.setVelocity(0)
    
    ultrasonic_sensors = []
    for sensor in ULTRASONIC_SENSORS:
        ultr_sens = robot.getDevice(sensor)
        ultr_sens.enable(TIME_STEP)
        ultrasonic_sensors.append(ultr_sens)
    
    infrared_sensors = []
    for sensor in INFRARED_SENSORS:
        infr_sens = robot.getDevice(sensor)
        infr_sens.enable(TIME_STEP)
        infrared_sensors.append(infr_sens)

    return robot, left_wheel, right_wheel, ultrasonic_sensors, infrared_sensors

def get_current_state(infrared_values):
    if infrared_values[9] > 750 and infrared_values[11] < 500:
        return 0  # Left line departure
    elif infrared_values[10] > 750 and infrared_values[8] < 500:
        return 1  # Right line departure
    else:
        return 2  # On the line or other cases

def update_refuerzo(refuerzo, action, prev_state, new_state, learning_rate):
    visitas[prev_state][action] += 1
    learning_rate = 1 / (1 + visitas[prev_state][action])
    Q[prev_state][action] = (1-learning_rate) * Q[prev_state][action] + learning_rate * (refuerzo + 0.5 * np.argmax(Q[new_state]))
    return learning_rate

def check_refuerzo(new_sensor_values, prev_sensor_values):
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

def perform_action(left_wheel, right_wheel, action):
    if action == 0:
        left_wheel.setVelocity(MAX_SPEED)    # Go straight
        right_wheel.setVelocity(MAX_SPEED)
    elif action == 1:
        left_wheel.setVelocity(MAX_SPEED)    # Turn right
        right_wheel.setVelocity(-MAX_SPEED)
    elif action == 2:
        left_wheel.setVelocity(-MAX_SPEED)   # Turn left
        right_wheel.setVelocity(MAX_SPEED)

def read_sensor_values(infrared_sensors):
    return [sensor.getValue() for sensor in infrared_sensors[8:12]]
   
def main():
    global last_second
    robot, leftWheel, rightWheel, ultrasonic_sensors, infrared_sensorss = init_devices()
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
    while robot.step(TIME_STEP) != -1:
        current_second = robot.getTime()
        if current_second != last_second:
            last_second = current_second
            
    
if __name__ == "__main__":
    main()
 

    
    
    
    
    
    