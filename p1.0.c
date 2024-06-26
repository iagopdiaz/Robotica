#pragma config(Sensor, S1,     touchSensor,    sensorEV3_Touch)
#pragma config(Sensor, S2,     gyroSensor,     sensorEV3_Gyro, modeEV3Gyro_RateAndAngle)
#pragma config(Sensor, S3,     colorSensor,    sensorEV3_Color, modeEV3Color_Ambient)
#pragma config(Sensor, S4,     sonarSensor,    sensorEV3_Ultrasonic)
#pragma config(Motor,  motorA,          armMotor,      tmotorEV3_Large, PIDControl, encoder)
#pragma config(Motor,  motorB,          leftMotor,     tmotorEV3_Large, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motorC,          rightMotor,    tmotorEV3_Large, PIDControl, driveRight, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

const int DIMS = 4;
int actuando = DIMS - DIMS; //0 no esta actuando nada, de 1 a Dims por prioridad luego
int prio[DIMS];
TSemaphore semaphore;

bool isInhibited(int prio){
	if ((actuando == 0) || (prio == actuando))
		return false;
	else
		return true;
}

bool takePrio(int prio){
	if ((actuando == 0) || (prio <= actuando)){
		actuando = prio;
		return true;
		}else{
		return false;
	}
}

void resetPrio(){
	actuando = 0;
}

void recto(){
	setMotorSpeed(motorB, 30);
	setMotorSpeed(motorC, 30);
}

void atras(){
	setMotorSpeed(motorB, -30);
	setMotorSpeed(motorC, -30);
}

void parar(){
	setMotorSpeed(motorB, 0);
	setMotorSpeed(motorC, 0);
}

void giro_drch(){
	setMotorSpeed(motorB, 30);
	setMotorSpeed(motorC, -30);
}

void giro_izq(){
	setMotorSpeed(motorB, -30);
	setMotorSpeed(motorC, 30);
}

int calcularVelocidad(int tiempoActual, int tiempoTotal) {
	int diferenciaMaxima = 14;
	// Calcula un factor que aumenta y luego disminuye, basado en la posición dentro del tiempo total
	int diferencia = diferenciaMaxima * sin(PI * tiempoActual / tiempoTotal);
	return diferencia;
}

task task_escapar() {
	bool mustAct = false;
	bool havePrio = false;
	while (true){
		semaphoreLock(semaphore);
		if (bDoesTaskOwnSemaphore(semaphore)) {
			semaphoreUnlock(semaphore);

			if (!isInhibited(prio[0])) {

				if (getTouchValue(S1) == 1 ){
					mustAct = true;
				}else
					mustAct = false;

				if (mustAct) {
					semaphoreLock(semaphore);
					havePrio = takePrio(prio[0]);
					semaphoreUnlock(semaphore);
					if (havePrio){
						clearTimer(T1);
						while(time10[T1] < 80){
							atras();
						}
						resetGyro(gyroSensor);
						giro_izq();
						while (abs(getGyroDegrees(gyroSensor)) < 60) {}
					}

					} else {
					semaphoreLock(semaphore);
					resetPrio();
					semaphoreUnlock(semaphore);
				}
			}

		}
	}
	abortTimeslice();
}

task task_luz{
	int limite = 10;
	int luz_max = 0;
	int luz_parar = 50;
	int angulo_ini = 0;
	bool mustAct = false;
	bool havePrio = false;

	while (true){
		semaphoreLock(semaphore);
		if (bDoesTaskOwnSemaphore(semaphore)) {
			semaphoreUnlock(semaphore);

			if (!isInhibited(prio[1])) {
				if (getColorAmbient(S3) > limite){
					mustAct = true;
					luz_max = getColorAmbient(S3);
				}else {
					mustAct = false;
				}
				if (mustAct) {
					semaphoreLock(semaphore);
					havePrio = takePrio(prio[1]);
					semaphoreUnlock(semaphore);
					resetGyro(gyroSensor);
					while (true) {
						while (abs(getGyroDegrees(gyroSensor)) < 360) {
							giro_drch();
							if (getColorAmbient(S3) > luz_max) {
								luz_max = getColorAmbient(S3);
								angulo_ini = getGyroDegrees(gyroSensor);
							}
						}
						resetGyro(gyroSensor);
						while (abs(getGyroDegrees(gyroSensor)) < angulo_ini) {
							giro_drch();
						}

						clearTimer(T1);
						while(time10[T1] < 150 && getColorAmbient(S3) < luz_parar){
							recto();
						}

						if(getColorAmbient(S3) >= luz_parar) {
							parar();
							break;
						}
					}
					} else {
					semaphoreLock(semaphore);
					resetPrio();
					semaphoreUnlock(semaphore);
				}

			}
		}
	}
	abortTimeslice();
}

task task_pared() {
	int	umbralDistancia = 20;
	int tiempoTotal = 500; // Tiempo total de la simulación
	int pasoTiempo = 50; // Intervalo de tiempo para ajustes de velocidad
	int tiempo = 0;
	int velocidadBase = 30; // Velocidad base
	int diferenciaVelocidad = 0;
	int v1 = 0;
	int v2 = 0;
	bool mustAct = false;
	bool havePrio = false;

	while (true){
		semaphoreLock(semaphore);
		if (bDoesTaskOwnSemaphore(semaphore)) {
			semaphoreUnlock(semaphore);

			if (!isInhibited(prio[2])) {
				if (getUSDistance(S4) < umbralDistancia){
					mustAct = true;
				}else
					mustAct = false;

				if (mustAct) {
					semaphoreLock(semaphore);
					havePrio = takePrio(prio[2]);
					semaphoreUnlock(semaphore);

					if (havePrio){
						tiempo = 0;
						setMotorSpeed(motorB,0);
						setMotorSpeed(motorC,0);

						clearTimer(T1);
						while(time10[T1] < 60){
							giro_izq();
						}

						clearTimer(T2);
						while(tiempo <= tiempoTotal){
							if (getUSDistance(S4) < umbralDistancia){
								break;
							}
							diferenciaVelocidad = calcularVelocidad(tiempo, tiempoTotal);
							v1 = velocidadBase + diferenciaVelocidad;
							v2 = velocidadBase - diferenciaVelocidad;
							setMotorSpeed(motorB,v1);
							setMotorSpeed(motorC,v2);

							tiempo = time10[T2];
						}
					}
					} else {
					semaphoreLock(semaphore);
					resetPrio();
					semaphoreUnlock(semaphore);
				}
			}
		}
	}
	abortTimeslice();
}

task task_recto (){
	while (true){
		semaphoreLock(semaphore);
		if (bDoesTaskOwnSemaphore(semaphore)) {
			semaphoreUnlock(semaphore);
			if (!isInhibited(prio[3])) {
				setMotorSpeed(motorB,30);
				setMotorSpeed(motorC,30);
			}
		}
		abortTimeslice();
	}
}

task main()
{
	prio[0] = 1;
	prio[1] = 2;
	prio[2] = 3;
	prio[3] = 4;

	semaphoreInitialize(semaphore);
	startTask(task_escapar);
	startTask(task_luz);
	startTask(task_pared);
	startTask(task_recto);


	while (true){
		abortTimeslice();
	}
}
