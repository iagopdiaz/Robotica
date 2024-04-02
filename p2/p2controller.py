
from controller import Robot, Motor, DistanceSensor, Camera

import math
import numpy as np
import cv2

# Odometría
TIME_STEP = 32
CRUISE_SPEED = 10
RADIO_RUEDA = 0.021
ESPACIO_ENTRE_RUEDAS = 0.10819
RADIO_ENTRE_RUEDAS = ESPACIO_ENTRE_RUEDAS/2

DELTA_RECTO = 0.25 / RADIO_RUEDA
DELTA_GIRO = (90 * (math.pi/180)) * RADIO_ENTRE_RUEDAS / RADIO_RUEDA

INICIOX = 1
INICIOY = 8
INICIO = INICIOX, INICIOY
TAMAÑO_MAPA = 12,12



mapa = np.zeros((TAMAÑO_MAPA), dtype=np.int8)

def inicializar_controladores(timestep):

    # create the Robot instance.
    robot = Robot()

    leftWheel = robot.getDevice("left wheel motor")
    rightWheel = robot.getDevice("right wheel motor")
    leftWheel.setPosition(0)
    rightWheel.setPosition(0)
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    posL = robot.getDevice("left wheel sensor")
    posR = robot.getDevice("right wheel sensor")
    posL.enable(timestep)
    posR.enable(timestep)

    camara = robot.getDevice("camera")
    camara.enable(timestep)

    sensores_infrarrojos = [
        "left infrared sensor",
        "front infrared sensor",
        "right infrared sensor",
    ]

    sensores_infrarrojos_init = []
    for nombre_sensor in sensores_infrarrojos:
        # Obtener el dispositivo sensor por su nombre
        sensor = robot.getDevice(nombre_sensor)
        # Activar el sensor
        sensor.enable(timestep)
        # Agregar el sensor a la lista de sensores infrarrojos
        sensores_infrarrojos_init.append(sensor)


    return robot, leftWheel, rightWheel, posL, posR, camara, sensores_infrarrojos_init

def stop_robot():
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

def avanzar(leftWheel, rightWheel, posL, posR,robot):
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    leftWheel.setPosition(posL.getValue() + DELTA_RECTO)
    rightWheel.setPosition(posR.getValue() + DELTA_RECTO)
    encoderL = posL.getValue() + DELTA_RECTO
    encoderR = posR.getValue() + DELTA_RECTO

    return encoderL, encoderR

def girar_derecha(leftWheel, rightWheel, posL, posR,robot):
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    leftWheel.setPosition(posL.getValue() + DELTA_GIRO)
    rightWheel.setPosition(posR.getValue() - DELTA_GIRO)
    encoderL = posL.getValue() + DELTA_GIRO
    encoderR = posR.getValue() - DELTA_GIRO

    return encoderL, encoderR

def girar_izquierda(leftWheel, rightWheel, posL, posR,robot):
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    leftWheel.setPosition(posL.getValue() - DELTA_GIRO)
    rightWheel.setPosition(posR.getValue() + DELTA_GIRO)
    encoderL = posL.getValue() - DELTA_GIRO
    encoderR = posR.getValue() + DELTA_GIRO

    return encoderL, encoderR

# No funciona aun
def seguir_paredes(leftWheel,rightWheel,posL,posR,Lista_sensores,robot):
     global orientacion
     if(DistanceSensor.getValue(Lista_sensores[0]) < 180):

         encoderL, encoderR = girar_izquierda(leftWheel, rightWheel, posL, posR,robot)
     elif(DistanceSensor.getValue(Lista_sensores[1]) < 180):

         encoderL, encoderR = avanzar(leftWheel, rightWheel, posL, posR,robot)
     elif(DistanceSensor.getValue(Lista_sensores[2]) >= 180):

         encoderL, encoderR = girar_derecha(leftWheel, rightWheel, posL, posR,robot)

def detectar_amarillo(camara):
    imagen = camara.getImage()
    altura = camara.getHeight()
    anchura = camara.getWidth()

    # Convertir la imagen a un array de numpy y luego a BGR
    imagen_np = np.frombuffer(imagen, np.uint8).reshape((altura, anchura, 4))
    imagen_bgr = cv2.cvtColor(imagen_np, cv2.COLOR_BGRA2BGR)

    # Convertir de BGR a HSV
    imagen_hsv = cv2.cvtColor(imagen_bgr, cv2.COLOR_BGR2HSV)

    # Rango de colores ajustado a lo que mencionaste
    amarillo_bajo = np.array([28, 100, 100], dtype=np.uint8)
    amarillo_alto = np.array([30, 255, 255], dtype=np.uint8)

    # Crear una máscara que solo incluye los colores dentro del rango amarillo
    mascara_amarillo = cv2.inRange(imagen_hsv, amarillo_bajo, amarillo_alto)

    # Calcular el porcentaje de píxeles amarillos
    pixeles_amarillos = cv2.countNonZero(mascara_amarillo)
    total_pixeles = altura * anchura
    porcentaje_amarillo = (pixeles_amarillos / total_pixeles) * 100

    umbral_cercania = 5

    if porcentaje_amarillo >= umbral_cercania:
        return True
    else:
        return False



def main():
    robot, leftWheel, rightWheel, posL, posR, camara, sensores_infrarrojos = inicializar_controladores(TIME_STEP)
    robot.step(TIME_STEP)
    while robot.step(TIME_STEP) != -1:

        seguir_paredes(leftWheel,rightWheel,posL,posR,sensores_infrarrojos,robot)
        """
            if detectar_amarillo(camara):
            # Acción a tomar cuando se detecta un objeto amarillo
            print("Objeto amarillo detectado")
            leftWheel.setVelocity(0)
            rightWheel.setVelocity(0)
            break
        else:
            # Continuar con el comportamiento normal
            leftWheel.setVelocity(CRUISE_SPEED)
            rightWheel.setVelocity(CRUISE_SPEED)
         """



if __name__ == "__main__":
    main()
