import numpy as np
from controller import Robot, Motor, DistanceSensor
import random

TIME_STEP = 32
MAX_SPEED = 6
GAMMA = 0.5
STATE_COUNT = 3
ACTION_COUNT = 3

Q = np.zeros((STATE_COUNT, ACTION_COUNT))
visits = np.zeros((STATE_COUNT, ACTION_COUNT))

current_state = 2
last_second = 0

learning_rate = 0.5
epsilon = 0.3  # Probabilidad de exploración inicial

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
    if infrared_values[1] > 750 and infrared_values[3] < 500:
        return 0  # Left line departure
    elif infrared_values[2] > 750 and infrared_values[0] < 500:
        return 1  # Right line departure
    
    return 2  # On the line or other cases
    
def update_refuerzo(refuerzo, action, prev_state, new_state):
    visits[prev_state][action] += 1
    learning_rate = 1 / (1 + visits[prev_state][action])
    Q[prev_state][action] = (1-learning_rate) * Q[prev_state][action] + learning_rate * (refuerzo + GAMMA * np.max(Q[new_state]))

    return learning_rate

def check_refuerzo(prev_sensor_values, new_sensor_values):
    refuerzo = 0
    indices = [0, 1, 2, 3]  # Indices for ground sensors

    for i in indices:
        if prev_sensor_values[i] < 500 and new_sensor_values[i] > 750:
            refuerzo -= 1  # Penalización por cambiar de negro a blanco
        elif prev_sensor_values[i] > 750 and new_sensor_values[i] < 500:
            refuerzo += 1  # Recompensa por cambiar de blanco a negro
        elif prev_sensor_values[i] < 500 and new_sensor_values[i] < 500:
            refuerzo += 0.5  # Recompensa por mantenerse en negro
        elif prev_sensor_values[i] > 750 and new_sensor_values[i] > 750:
            refuerzo -= 0.5  # Penalización por mantenerse en blanco

    return refuerzo

def perform_action(left_wheel, right_wheel, action):
    if action == 0:
        left_wheel.setVelocity(MAX_SPEED)    # Turn right
        right_wheel.setVelocity(-MAX_SPEED)
    elif action == 1:
        left_wheel.setVelocity(-MAX_SPEED)   # Turn left
        right_wheel.setVelocity(MAX_SPEED)
    elif action == 2:
        left_wheel.setVelocity(MAX_SPEED)    # Go straight
        right_wheel.setVelocity(MAX_SPEED)
    
def read_sensor_values(infrared_sensors):
    return [sensor.getValue() for sensor in infrared_sensors[8:12]]

def main():
    global last_second, current_state, learning_rate, epsilon
    robot, leftWheel, rightWheel, ultrasonic_sensors, infrared_sensors = init_devices()
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
    while robot.step(TIME_STEP) != -1:
        current_second = robot.getTime()
        if current_second != last_second:
            last_second = current_second

        if (infrared_sensors[2].getValue() > 300 or infrared_sensors[3].getValue() > 300 or infrared_sensors[4].getValue() > 300):
            speed_offset = 0.2 * (MAX_SPEED - 0.03 * infrared_sensors[3].getValue())
            speed_delta = 0.03 * infrared_sensors[2].getValue() - 0.03 * infrared_sensors[4].getValue()
            leftWheel.setVelocity(speed_offset + speed_delta)
            rightWheel.setVelocity(speed_offset - speed_delta)
        else:
            current_sensor_values = read_sensor_values(infrared_sensors)
            current_state = get_current_state(current_sensor_values)

            if random.uniform(0, 1) < epsilon:
                action = random.choice([0, 1, 2])
            else:
                action = np.argmax(Q[current_state])
            
            perform_action(leftWheel, rightWheel, action)
            print(Q)

            new_sensor_values = read_sensor_values(infrared_sensors)
            new_state = get_current_state(new_sensor_values)

            refuerzo = check_refuerzo(current_sensor_values, new_sensor_values)
            learning_rate = update_refuerzo(refuerzo, action, current_state, new_state)
               

if __name__ == "__main__":
    main()
