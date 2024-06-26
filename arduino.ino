#include <Servo.h>
#include <NHCSR04.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Adafruit_APDS9960.h"

#define TRIG_PIN 13
#define ECHO_PIN 12
#define SERVO_PIN 7
#define MOTOR_A1 5
#define MOTOR_A2 6
#define MOTOR_B1 9
#define MOTOR_B2 10
#define MAX_DISTANCE 200  // Máximo rango de medición
#define SPEED_OF_SOUND 0.0343  // Puede ajustarse según la calibración
#define RxD 3
#define TxD 2

char leer;

SoftwareSerial BTSerial(RxD, TxD); // RX | TX
Servo myservo;  // Crear objeto de servo
SR04 sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Adafruit_APDS9960 apds;

double Setpoint, Input, Output;
double kp = 0.2, ki = 0.2, kd = 0.4;
double integral = 0, last_error = 0;
unsigned long lastTime;
double errSum, lastErr;
int motorSpeed = 200; // Rango de 0 a 255, donde 255 es la máxima velocidad

void setup() {
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  myservo.attach(SERVO_PIN); // Asignar el pin al servo
  Serial.begin(9600);
  BTSerial.begin(9600);
  Setpoint = 200; // Distancia deseada en cm

  if (!apds.begin()) {
    Serial.println("failed to initialize device! Please check your wiring.");
    while (1); // Detener el programa si no se encuentra el sensor
  } else {
    Serial.println("Device initialized!");
  }

  // Enable color sensing mode
  apds.enableColor(true);

  lastTime = millis(); // Inicializar lastTime
}

void loop() {
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  int distance = sonar.centimeters(); // Medir distancia
  Input = distance;
  double error = Setpoint - Input;
  errSum += (error * timeChange);
  double dErr = (error - lastErr) / timeChange;

  // Calculo del PID
  Output = kp * error + ki * errSum + kd * dErr;

  // Mapear y escribir al servo
  //int servoPos = map(Output, -10, 10, 40, 140); // Ajustar estos valores según necesidad
  //Serial.print("servo=");
  //Serial.println(constrain(servoPos, 40, 140));
  //myservo.write(constrain(servoPos, 40, 140));

  lastErr = error;
  lastTime = now;
  Serial.print("Distancia=");
  Serial.println(distance);

  if (distance > 0 && distance < 10) { // Si detecta un obstáculo cerca|
    stopMotors(); // Detener motores
    delay(500);
    int direction = random(0, 2); // Generar un número aleatorio 0 o 1
    while (distance > 0 && distance < 10) {
      if (direction == 1) {
        turnLeft(); // Mover a la izquierda
      } else {
        turnRight(); // Mover a la derecha
      }
      delay(500); // Esperar un poco antes de verificar nuevamente
      distance = sonar.centimeters(); // Medir distancia nuevamente
      Serial.print("Distancia (moviendo) = ");
      Serial.println(distance);
    }
    stopMotors(); // Detener motores una vez que el obstáculo ya no esté en frente
  }

  // Verificar si ya no hay obstáculos y avanzar
  if (distance >= 20) {
    Serial.println("adelante");
    turnLeft(); // Mover hacia adelante
  }

  // Leer datos del sensor de color
  uint16_t r, g, b, c;
  if (apds.colorDataReady()) {
    apds.getColorData(&r, &g, &b, &c);
    Serial.print("COLOR ");
    Serial.print(r); Serial.print(" ");
    Serial.print(g); Serial.print(" ");
    Serial.print(b); Serial.print(" ");
    Serial.print(c);
    Serial.print(" POS ");
    //Serial.println(servoPos);
  }

  delay(1000);

  // Leer puerto serial
  if (BTSerial.available()) { // leer desde HC-05 y enviar al Monitor Serial de Arduino
    leer = BTSerial.read();
    Serial.println(leer);

    // Mover el robot según el comando recibido
    if (leer == 'T') {
      forward();
    } else if (leer == 'S') {
      stopMotors();
    } else if (leer == 'L') {
      turnLeft();
    } else if (leer == 'R') {
      turnRight();
    }
  }
}

void turnRight() {
  digitalWrite(MOTOR_A1, 0);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, 0);
}

void stopMotors() {
  digitalWrite(MOTOR_A1, 0);
  digitalWrite(MOTOR_A2, 0);
  digitalWrite(MOTOR_B1, 0);
  digitalWrite(MOTOR_B2, 0);
}

void turnLeft() {
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, 0);
  digitalWrite(MOTOR_B1, 0);
  digitalWrite(MOTOR_B2, HIGH);
}

void forward() {
 digitalWrite(MOTOR_A1, HIGH);
 digitalWrite(MOTOR_A2, LOW);
 digitalWrite(MOTOR_B1, HIGH);
 digitalWrite(MOTOR_B2, LOW);
}