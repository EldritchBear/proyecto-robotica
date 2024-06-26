#include <Servo.h>
#include <NewPing.h>
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
#define GRID_SIZE 5
#define MAX_PATH_LENGTH 20

char leer;

SoftwareSerial BTSerial(RxD, TxD); // RX | TX
Servo myservo;  // Crear objeto de servo
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
Adafruit_APDS9960 apds;

double Setpoint, Input, Output;
double kp = 0.2, ki = 0.2, kd = 0.4;
double integral = 0, last_error = 0;
unsigned long lastTime;
double errSum, lastErr;
int motorSpeed = 200; // Rango de 0 a 255, donde 255 es la máxima velocidad

int grid[GRID_SIZE][GRID_SIZE];
int path[MAX_PATH_LENGTH][2];
int pathLength = 0;
int pathIndex = 0;

int startX = 0, startY = 0; // Variables globales para la posición inicial

struct Node {
  int x, y;
  float g, h;
  Node* parent;
};

Node nodes[GRID_SIZE][GRID_SIZE];
Node* openList[GRID_SIZE * GRID_SIZE];
int openListLength = 0;
Node* closedList[GRID_SIZE * GRID_SIZE];
int closedListLength = 0;

void addToOpenList(Node* node) {
  openList[openListLength++] = node;
}

void removeFromOpenList(Node* node) {
  for (int i = 0; i < openListLength; i++) {
    if (openList[i] == node) {
      for (int j = i; j < openListLength - 1; j++) {
        openList[j] = openList[j + 1];
      }
      openListLength--;
      break;
    }
  }
}

bool isInClosedList(Node* node) {
  for (int i = 0; i < closedListLength; i++) {
    if (closedList[i] == node) {
      return true;
    }
  }
  return false;
}

void addToClosedList(Node* node) {
  closedList[closedListLength++] = node;
}

Node* getLowestFNode() {
  Node* lowest = openList[0];
  for (int i = 1; i < openListLength; i++) {
    if (openList[i]->g + openList[i]->h < lowest->g + lowest->h) {
      lowest = openList[i];
    }
  }
  return lowest;
}

void reconstructPath(Node* node) {
  while (node != nullptr) {
    path[pathLength][0] = node->x;
    path[pathLength][1] = node->y;
    pathLength++;
    node = node->parent;
  }
}

float heuristic(int x, int y, int endX, int endY) {
  return abs(x - endX) + abs(y - endY);
}

void aStar(int startX, int startY, int endX, int endY) {
  for (int x = 0; x < GRID_SIZE; x++) {
    for (int y = 0; y < GRID_SIZE; y++) {
      nodes[x][y] = { x, y, INFINITY, INFINITY, nullptr };
    }
  }
  nodes[startX][startY].g = 0;
  nodes[startX][startY].h = heuristic(startX, startY, endX, endY);

  addToOpenList(&nodes[startX][startY]);

  while (openListLength > 0) {
    Node* current = getLowestFNode();
    if (current->x == endX && current->y == endY) {
      reconstructPath(current);
      return;
    }

    removeFromOpenList(current);
    addToClosedList(current);

    Node* neighbors[4];
    int neighborCount = 0;

    if (current->x > 0) neighbors[neighborCount++] = &nodes[current->x - 1][current->y];
    if (current->x < GRID_SIZE - 1) neighbors[neighborCount++] = &nodes[current->x + 1][current->y];
    if (current->y > 0) neighbors[neighborCount++] = &nodes[current->x][current->y - 1];
    if (current->y < GRID_SIZE - 1) neighbors[neighborCount++] = &nodes[current->x][current->y + 1];

    for (int i = 0; i < neighborCount; i++) {
      Node* neighbor = neighbors[i];
      if (isInClosedList(neighbor) || grid[neighbor->x][neighbor->y] == 1) {
        continue;
      }

      float tentativeG = current->g + 1;
      bool isInOpenList = false;
      for (int j = 0; j < openListLength; j++) {
        if (openList[j] == neighbor) {
          isInOpenList = true;
          break;
        }
      }

      if (!isInOpenList || tentativeG < neighbor->g) {
        neighbor->parent = current;
        neighbor->g = tentativeG;
        neighbor->h = heuristic(neighbor->x, neighbor->y, endX, endY);
        if (!isInOpenList) {
          addToOpenList(neighbor);
        }
      }
    }
  }
}

void moveToNextCell() {
  if (pathIndex >= pathLength) return;
  int nextX = path[pathIndex][0];
  int nextY = path[pathIndex][1];

  if (nextX > startX) {
    turnRight();
  } else if (nextX < startX) {
    turnLeft();
  } else if (nextY > startY) {
    forward();
  }

  startX = nextX;
  startY = nextY;
  pathIndex++;
}

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

  // Inicializar la cuadrícula con celdas libres (0) y obstáculos (1)
  for (int x = 0; x < GRID_SIZE; x++) {
    for (int y = 0; y < GRID_SIZE; y++) {
      grid[x][y] = 0; // Suponer todas las celdas están libres inicialmente
    }
  }

  // Definir obstáculos en la cuadrícula (ejemplo)
  grid[2][2] = 1;
  grid[2][3] = 1;
  grid[3][2] = 1;

  aStar(0, 0, GRID_SIZE - 1, GRID_SIZE - 1); // Calcula la ruta una vez al inicio
}

void loop() {
  if (pathIndex < pathLength) {
    moveToNextCell(); // Mueve al siguiente nodo en el camino
  } else {
    Serial.println("No path found or destination reached!");
  }

  delay(1000); // Esperar un poco antes de verificar nuevamente
}

void turnLeft() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, motorSpeed);
  analogWrite(MOTOR_B1, motorSpeed);
  analogWrite(MOTOR_B2, 0);
  delay(500);
  stopMotors();
}

void stopMotors() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}

void turnRight() {
  analogWrite(MOTOR_A1, motorSpeed);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, motorSpeed);
  delay(500);
  stopMotors();
}
void forward() {
  analogWrite(MOTOR_A1, motorSpeed);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, motorSpeed);
  analogWrite(MOTOR_B2, 0);
  delay(500);
  stopMotors();
}
