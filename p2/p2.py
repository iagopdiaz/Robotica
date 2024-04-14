
from controller import Robot, Motor, DistanceSensor, Camera
import time 
import math
import numpy as np
import cv2

# Odometría
TIME_STEP = 32
CRUISE_SPEED = 10
RADIO_RUEDA = 21
ESPACIO_ENTRE_RUEDAS = 107.90
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

coordsDiag = [
    ( 1, 1),
    (1, -1),  
    (-1, -1),  
    (-1, 1)  
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
        "front right infrared sensor",
        "rear right infrared sensor",
        "rear left infrared sensor",
        "front left infrared sensor"       
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
    mapa[TAMAÑO_MAPA - 1, TAMAÑO_MAPA - 1] = -1
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

    distDelante = DistanceSensor.getValue(Lista_sensores[1])
    distDrch = DistanceSensor.getValue(Lista_sensores[2])
    distDetras = DistanceSensor.getValue(Lista_sensores[3])
    distIzq = DistanceSensor.getValue(Lista_sensores[0])

    if (distDelante >= 180):                                         #Delante
        (sumx,sumy) = coords[sum1]
        if (mapa[x + sumx, y + sumy] == 0):
            mapa[x + sumx, y + sumy] = 1

    if (distDrch >= 180):                                             #Drch
        (sumx,sumy) = coords[sum2]
        if (mapa[x + sumx, y + sumy] == 0):
            mapa[x + sumx, y + sumy] = 1

    if (distDetras  >= 180):                                          #Detras
        (sumx,sumy) = coords[sum3]
        if (mapa[x + sumx, y + sumy] == 0):
            mapa[x + sumx, y + sumy] = 1
        
    if (distIzq >= 180):                                               #Izq
        (sumx,sumy) = coords[sum4]
        if (mapa[x + sumx, y + sumy] == 0):
            mapa[x + sumx, y + sumy] = 1

    if (distDelante >= 180 and distDrch >= 180):                    #Delante-Drch
        (sumx,sumy) = coordsDiag[sum1]
        if (mapa[x + sumx, y + sumy] == 0):
            mapa[x + sumx, y + sumy] = 1

    if (distDetras >= 180 and distDrch >= 180):                    #Detras-Drch
        (sumx,sumy) = coordsDiag[sum2]
        if (mapa[x + sumx, y + sumy] == 0):
            mapa[x + sumx, y + sumy] = 1

    if (distDetras >= 180 and distIzq >= 180):                    #Detras-Izq
        (sumx,sumy) = coordsDiag[sum3]
        if (mapa[x + sumx, y + sumy] == 0):
            mapa[x + sumx, y + sumy] = 1
    
    if (distDelante >= 180 and distIzq >= 180):                    #Delante-Izq
        (sumx,sumy) = coordsDiag[sum4]
        if (mapa[x + sumx, y + sumy] == 0):
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

def avanzar(leftWheel, rightWheel, posL, posR, pos_robot, mirando, robot):
    leftWheel.setVelocity(CRUISE_SPEED)
    rightWheel.setVelocity(CRUISE_SPEED)
    leftWheel.setPosition(posL.getValue() + DELTA_RECTO)
    rightWheel.setPosition(posR.getValue() + DELTA_RECTO)
    encoderL = posL.getValue() + DELTA_RECTO
    encoderR = posR.getValue() + DELTA_RECTO

    while not(posL.getValue() >= encoderL - 0.004 and posR.getValue() >= encoderR - 0.004):  
        robot.step(TIME_STEP)
        
    new_pos = cambiarPos(pos_robot, mirando)

    return encoderL, encoderR, new_pos

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
    while not(posL.getValue() >= encoderL - 0.004 and posR.getValue() <= encoderR + 0.004):  
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
    while not(posL.getValue() <= encoderL + 0.004 and posR.getValue() >= encoderR - 0.004):  
        robot.step(TIME_STEP)

    mirando = cambiarMirando(-1, mirando)

    return encoderL, encoderR, mirando


def seguir_paredes(leftWheel, rightWheel, posL, posR, Lista_sensores, mapa, pos_robot, mirando, robot):
    global orientacion
     
    if(DistanceSensor.getValue(Lista_sensores[1]) <= 140 and DistanceSensor.getValue(Lista_sensores[0]) >= 150):
        encoderL, encoderR, pos_robot = avanzar(leftWheel, rightWheel, posL, posR, pos_robot, mirando, robot)

    elif(DistanceSensor.getValue(Lista_sensores[0]) < 150 and DistanceSensor.getValue(Lista_sensores[1]) < 150):
        encoderL, encoderR, mirando = girar_izquierda(leftWheel, rightWheel, posL, posR, mirando, robot)
        encoderL, encoderR, pos_robot = avanzar(leftWheel, rightWheel, posL, posR, pos_robot, mirando, robot)
        
    else:
        encoderL, encoderR, mirando = girar_derecha(leftWheel, rightWheel, posL, posR, mirando, robot)

    return pos_robot, mirando

def encontrar_siguiente_paso(distancias, pos_robot):
    # Determina la posición adyacente con la menor distancia hacia la base
    x, y = pos_robot
    min_dist = float('inf')
    siguiente_paso = None
    movimientos = [(0, 1), (0, -1), (-1, 0), (1, 0)]  # Arriba, abajo, izquierda, derecha

    for dx, dy in movimientos:
        nx, ny = x + dx, y + dy
        if 0 <= nx < distancias.shape[0] and 0 <= ny < distancias.shape[1]:
            if distancias[nx, ny] < min_dist:
                min_dist = distancias[nx, ny]
                siguiente_paso = (nx, ny)

    return siguiente_paso

def volver_base(leftWheel, rightWheel, posL, posR, mapa_vuelta, pos_robot, mirando, robot):
    
    for siguiente_paso in mapa_vuelta:   
    # Decide si necesita girar o avanzar basado en la posición del siguiente paso
        if siguiente_paso == (pos_robot[0], pos_robot[1] + 1):  # Si el siguiente paso es arriba
            while(mirando != 0):
                if (mirando == 1):
                    encoderL, encoderR, mirando = girar_izquierda(leftWheel, rightWheel, posL, posR, mirando, robot)
                else:
                    encoderL, encoderR, mirando = girar_derecha(leftWheel, rightWheel, posL, posR, mirando, robot)
            encoderL, encoderR, pos_robot = avanzar(leftWheel, rightWheel, posL, posR, pos_robot, mirando, robot)
            pass

        elif siguiente_paso == (pos_robot[0] + 1, pos_robot[1]):  # Si el siguiente paso es a la derecha
            while(mirando != 1):
                if (mirando == 2):
                    encoderL, encoderR, mirando = girar_izquierda(leftWheel, rightWheel, posL, posR, mirando, robot)
                else:
                    encoderL, encoderR, mirando = girar_derecha(leftWheel, rightWheel, posL, posR, mirando, robot)
            encoderL, encoderR, pos_robot = avanzar(leftWheel, rightWheel, posL, posR, pos_robot, mirando, robot)
            pass

        elif siguiente_paso == (pos_robot[0], pos_robot[1] - 1):  # Si el siguiente paso es abajo
            while(mirando != 2):
                if (mirando == 3):
                    encoderL, encoderR, mirando = girar_izquierda(leftWheel, rightWheel, posL, posR, mirando, robot)
                else:
                    encoderL, encoderR, mirando = girar_derecha(leftWheel, rightWheel, posL, posR, mirando, robot)
            encoderL, encoderR, pos_robot = avanzar(leftWheel, rightWheel, posL, posR, pos_robot, mirando, robot)
            pass

        elif siguiente_paso == (pos_robot[0] - 1, pos_robot[1]):  # Si el siguiente paso es a la izquierda
            while(mirando != 3):
                if (mirando == 0):
                    encoderL, encoderR, mirando = girar_izquierda(leftWheel, rightWheel, posL, posR, mirando, robot)
                else:
                    encoderL, encoderR, mirando = girar_derecha(leftWheel, rightWheel, posL, posR, mirando, robot)
            encoderL, encoderR, pos_robot = avanzar(leftWheel, rightWheel, posL, posR, pos_robot, mirando, robot)
            pass
    

        # Actualiza pos_robot después de avanzar
        pos_robot = siguiente_paso

    return pos_robot, mirando

def selec_movimiento(nx, ny, movimiento_seleccionado, menor_distancia, distancias):
    if 0 <= nx < distancias.shape[0] and 0 <= ny < distancias.shape[1] and distancias[nx, ny] != 100:
        if distancias[nx, ny] <= menor_distancia:
            menor_distancia = distancias[nx, ny]
            movimiento_seleccionado = (nx, ny)

    return movimiento_seleccionado

def calcular_mapa_vuelta(pos_robot, distancias):
    # Extraer la posición inicial del robot
    x, y = pos_robot
    camino = []  # Iniciar el camino con la posición actual del robot
    

    # Seguir el camino hasta llegar a la base
    while distancias[x, y] != 0:
        movimiento_seleccionado = None
        menor_distancia = distancias[x, y]  # Inicia con la distancia actual

        for (dx, dy) in coords:
            nx, ny = x + dx, y + dy
            if not any((x == nx and y == ny) for x, y in camino):
                    movimiento_seleccionado = selec_movimiento(nx, ny, movimiento_seleccionado, menor_distancia, distancias)

        # Verificar que siempre hay un movimiento válido
        if movimiento_seleccionado is None:
            for (dx, dy) in coords:
                nx, ny = x + dx, y + dy
                if not any((x == nx and y == ny) for x, y in camino):
                    movimiento_seleccionado = selec_movimiento(nx, ny, movimiento_seleccionado, menor_distancia + 1, distancias)
                
        # Actualizar la posición del robot al movimiento seleccionado
        x, y = movimiento_seleccionado
        print(movimiento_seleccionado)
        camino.append(movimiento_seleccionado)

    return camino

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

    umbral_cercania = 10

    if porcentaje_amarillo >= umbral_cercania:
        return True
    else:
        return False

def expandir_frente_de_onda(mapa, base_pos):

    distancias = np.full(mapa.shape, 200)
    
    # Establece la distancia en la base a 0 y en las paredes a 100
    distancias[base_pos] = 0
    distancias[mapa == 1] = 100

    # Cola para los puntos a procesar, comienza con la posición de la base
    cola = [base_pos]
    
    # Procesa la cola
    while cola:
        x, y = cola.pop(0)
        for (dx, dy) in coords:
            nx, ny = x + dx, y + dy
            # Verifica si la nueva posición está dentro de los límites del mapa
            if 0 <= nx < mapa.shape[0] and 0 <= ny < mapa.shape[1]:
                # Permitir que la onda pase a través de la pared sin aumentar el costo
                if mapa[nx, ny] != 1 and distancias[nx, ny] > distancias[x, y] + 1:
                    distancias[nx, ny] = distancias[x, y] + 1
                    cola.append((nx, ny))
                elif mapa[nx, ny] == 1 and distancias[nx, ny] > distancias[x, y] + 1:
                    distancias[nx, ny] = distancias[x, y] + 1  # Tratar la pared como parte del camino sin cambiar su etiqueta finalmente
                    cola.append((nx, ny))

    # Restaurar la distancia de las paredes a 100 después de propagar
    distancias[mapa == 1] = 100

                                
    return distancias

def main():
    robot, leftWheel, rightWheel, posL, posR, camara, sensores_infrarrojos, mapa, pos_robot, mirando = inicializar_controladores(TIME_STEP)
    robot.step(TIME_STEP)
    time.sleep(1)
    mapa = guardarMapa(sensores_infrarrojos, mapa, pos_robot, mirando)
    pos_pasada = pos_robot

    #Bucle para salir de Base antes del bucle principal
    while robot.step(TIME_STEP) != -1:
        pos_robot, mirando = seguir_paredes(leftWheel,rightWheel,posL,posR,sensores_infrarrojos, mapa, pos_robot, mirando, robot)
        mapa = guardarMapa(sensores_infrarrojos, mapa, pos_robot, mirando)

        if detectar_amarillo(camara):
            # Acción a tomar cuando se detecta un objeto amarillo
            (x, y) = pos_robot
            (sumx, sumy) = coords[mirando]
            mapa[x + sumx, y + sumy] = 2  

        if (pos_robot != pos_pasada):   break
        else:   pos_pasada = pos_robot

    #Bucle Mapear
    (x1, y1) = pos_robot
    while (mapa[x1,y1] != -1):                         
        robot.step(TIME_STEP)
        pos_robot, mirando = seguir_paredes(leftWheel,rightWheel,posL,posR,sensores_infrarrojos, mapa, pos_robot, mirando, robot)
        mapa = guardarMapa(sensores_infrarrojos, mapa, pos_robot, mirando)
        if detectar_amarillo(camara):
            (x, y) = pos_robot
            # Acción a tomar cuando se detecta un objeto amarillo
            (sumx, sumy) = coords[mirando]
            mapa[x + sumx, y + sumy] = 2  
        (x1, y1) = pos_robot  
             
    
    distancias = expandir_frente_de_onda(mapa, (TAMAÑO_MAPA-1, TAMAÑO_MAPA-1))
    np.savetxt('./mapa.txt', mapa, fmt='%d')
    np.savetxt('./distancias.txt', distancias, fmt='%d')
    print(distancias)

    #Bucle Buscar amarillo
    while (not detectar_amarillo(camara)):
        robot.step(TIME_STEP)            
        print(f"Buscando...")
        pos_robot, mirando = seguir_paredes(leftWheel,rightWheel,posL,posR,sensores_infrarrojos, mapa, pos_robot, mirando, robot)

    mapa_vuelta = calcular_mapa_vuelta(pos_robot, distancias)
    (x1, y1) = pos_robot
    while (mapa[x1,y1] != -1):                         
        robot.step(TIME_STEP)
        print(f"Volviendo...")
        pos_robot, mirando = volver_base(leftWheel, rightWheel, posL, posR, mapa_vuelta, pos_robot, mirando, robot)  
        (x1, y1) = pos_robot


if __name__ == "__main__":
    main()
