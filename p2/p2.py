
from controller import Robot, Motor, DistanceSensor, Camera
import time 
import math
import numpy as np
import cv2

# Odometría
TIME_STEP = 32
CRUISE_SPEED = 10
RADIO_RUEDA = 21
ESPACIO_ENTRE_RUEDAS = 108.19
RADIO_ENTRE_RUEDAS = ESPACIO_ENTRE_RUEDAS/2

DELTA_RECTO = 250 / RADIO_RUEDA
DELTA_GIRO = (90 * (math.pi/180)) * RADIO_ENTRE_RUEDAS / RADIO_RUEDA

INICIOX = 1
INICIOY = 8
INICIO = INICIOX, INICIOY
TAMAÑO_MAPA = 12

coords = [
    (0, 1),
    (1, 0),  
    (0, -1),  
    (-1,0)  
]

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
    camara.enable(120)

    sensores_infrarrojos = [
        "left infrared sensor",
        "front infrared sensor",
        "right infrared sensor",
        "rear infrared sensor",
        "front left infrared sensor",
        "front right infrared sensor",
        "rear left infrared sensor",
        "rear right infrared sensor",
    ]

    sensores_infrarrojos_init = []
    for nombre_sensor in sensores_infrarrojos:
        # Obtener el dispositivo sensor por su nombre
        sensor = robot.getDevice(nombre_sensor)
        # Activar el sensor
        sensor.enable(timestep)
        # Agregar el sensor a la lista de sensores infrarrojos
        sensores_infrarrojos_init.append(sensor)

    mapa = np.zeros((TAMAÑO_MAPA*2, TAMAÑO_MAPA*2), dtype=np.int8)
    pos_robot = (TAMAÑO_MAPA - 1, TAMAÑO_MAPA - 1)
    mirando = 0

    return robot, leftWheel, rightWheel, posL, posR, camara, sensores_infrarrojos_init, mapa, pos_robot, mirando

def guardarMapa(Lista_sensores, mapa, pos_robot, mirando):
    (x, y) = pos_robot

    if mirando == 0:                                                   #Si se va hacia arriba
        sum1 = 0
        sum2 = 1
        sum3 = 2
        sum4 = 3
    elif mirando == 1:                                                 #Si se va hacia drch
        sum1 = 1  
        sum2 = 2
        sum3 = 3
        sum4 = 0
    elif mirando == 2:                                                 #Si se va hacia abajo
        sum1 = 2
        sum2 = 3
        sum3 = 0
        sum4 = 1
    elif mirando == 3:                                                 #Si se va hacia izq
        sum1 = 3
        sum2 = 0
        sum3 = 1
        sum4 = 2

    if (DistanceSensor.getValue(Lista_sensores[1]) >= 180):            #Delante
        (sumx,sumy) = coords[sum1]
        mapa[x + sumx, y + sumy] = 1

    if (DistanceSensor.getValue(Lista_sensores[2]) >= 180):            #Drch
        (sumx,sumy) = coords[sum2]
        mapa[x + sumx, y + sumy] = 1

    if (DistanceSensor.getValue(Lista_sensores[3]) >= 180):            #Detras
        (sumx,sumy) = coords[sum3]
        mapa[x + sumx, y + sumy] = 1
        
    if (DistanceSensor.getValue(Lista_sensores[0]) >= 180):            #Izq
        (sumx,sumy) = coords[sum4]
        mapa[x + sumx, y + sumy] = 1

    
    

    return mapa

def cambiarPos(pos_robot, mirando):
    (x, y) = pos_robot

    if mirando == 0: 
        pos_robot = (x, y+1)

    elif mirando == 1:                                   
        pos_robot = (x+1, y)

    elif mirando == 2: 
        pos_robot = (x, y-1)

    elif mirando == 3:                                  
        pos_robot = (x-1, y)

    return pos_robot

def cambiarMirando(mirar, mirando):
    if mirar == 1:              #Al girar a la drch
        mirando += 1
    elif mirar == -1:           #Al girar a la izq
        mirando -= 1

    if mirando == 4:          #Si se completa un giro desde la izq se vuelve a 0, mirar arriba
        mirando = 0
    elif mirando == -1:       #Si al girar desde mirando delante se mira a la izq, se mira a 3
        mirando = 3

    return mirando

def stop_robot():
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

def avanzar(leftWheel, rightWheel, posL, posR, Lista_sensores, mapa, pos_robot, mirando, robot):
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    leftWheel.setPosition(posL.getValue() + DELTA_RECTO)
    rightWheel.setPosition(posR.getValue() + DELTA_RECTO)
    encoderL = posL.getValue() + DELTA_RECTO
    encoderR = posR.getValue() + DELTA_RECTO

    while not(posL.getValue() >= encoderL - 0.001 and posR.getValue() >= encoderR - 0.001):  
        robot.step(TIME_STEP)
    
    new_pos = cambiarPos(pos_robot, mirando)
    mapa = guardarMapa(Lista_sensores, mapa, new_pos, mirando)

    return encoderL, encoderR, new_pos, mapa

def girar_derecha(leftWheel, rightWheel, posL, posR, mirando, robot):
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    encoderL = posL.getValue() + DELTA_GIRO  # Calcula el objetivo para la rueda izquierda
    encoderR = posR.getValue() - DELTA_GIRO  # Calcula el objetivo para la rueda derecha
    leftWheel.setVelocity(CRUISE_SPEED)  # Establece la velocidad para comenzar el giro
    rightWheel.setVelocity(CRUISE_SPEED)  # Asegúrate de que este valor sea negativo para girar a la derecha
    
    # Establece las posiciones objetivo sin esperar un valor de retorno
    leftWheel.setPosition(encoderL)
    rightWheel.setPosition(encoderR)

    # Espera hasta que el robot alcance las posiciones objetivo
    while not(posL.getValue() >= encoderL - 0.001 and posR.getValue() <= encoderR + 0.001):  
        robot.step(TIME_STEP)
    
    mirando = cambiarMirando(1, mirando)

    return encoderL, encoderR, mirando

def girar_izquierda(leftWheel, rightWheel, posL, posR, mirando, robot):
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    encoderL = posL.getValue() - DELTA_GIRO  # Calcula el objetivo para la rueda izquierda
    encoderR = posR.getValue() + DELTA_GIRO  # Calcula el objetivo para la rueda derecha
    leftWheel.setVelocity(CRUISE_SPEED)  # Establece la velocidad para comenzar el giro
    rightWheel.setVelocity(CRUISE_SPEED)  # Asegúrate de que este valor sea negativo para girar a la derecha
    
    # Establece las posiciones objetivo sin esperar un valor de retorno
    leftWheel.setPosition(encoderL)
    rightWheel.setPosition(encoderR)

    # Espera hasta que el robot alcance las posiciones objetivo
    while not(posL.getValue() <= encoderL + 0.001 and posR.getValue() >= encoderR - 0.001):  
        robot.step(TIME_STEP)

    mirando = cambiarMirando(-1, mirando)

    return encoderL, encoderR, mirando


def seguir_paredes(leftWheel, rightWheel, posL, posR, Lista_sensores, mapa, pos_robot, mirando, robot):
    global orientacion
     
    if(DistanceSensor.getValue(Lista_sensores[1]) <= 140 and DistanceSensor.getValue(Lista_sensores[0]) >= 150):
        encoderL, encoderR, pos_robot, mapa = avanzar(leftWheel, rightWheel, posL, posR, Lista_sensores, mapa, pos_robot, mirando, robot)

    elif(DistanceSensor.getValue(Lista_sensores[0]) < 150 and DistanceSensor.getValue(Lista_sensores[1]) < 150 and DistanceSensor.getValue(Lista_sensores[2]) < 150 and DistanceSensor.getValue(Lista_sensores[3]) < 150):
        encoderL, encoderR, mirando = girar_izquierda(leftWheel, rightWheel, posL, posR, mirando, robot)
        encoderL, encoderR, pos_robot, mapa = avanzar(leftWheel, rightWheel, posL, posR, Lista_sensores, mapa, pos_robot, mirando, robot)

    else:
        encoderL, encoderR, mirando = girar_derecha(leftWheel, rightWheel, posL, posR, mirando, robot)
        
    return mapa, pos_robot, mirando

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
    robot, leftWheel, rightWheel, posL, posR, camara, sensores_infrarrojos, mapa, pos_robot, mirando = inicializar_controladores(TIME_STEP)
    robot.step(TIME_STEP)
    time.sleep(1)
    mapa = guardarMapa(sensores_infrarrojos, mapa, pos_robot, mirando)
    while robot.step(TIME_STEP) != -1:
        print(f"pos: {pos_robot}")
        print(f"mirando: {mirando}")
        print(f"mapa: {mapa}")
        mapa, pos_robot, mirando = seguir_paredes(leftWheel,rightWheel,posL,posR,sensores_infrarrojos, mapa, pos_robot, mirando, robot)
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
        
        np.savetxt('./mapa.txt', mapa, fmt='%d')


if __name__ == "__main__":
    main()
