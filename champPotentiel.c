//TP EA08 ROBOT PIXY PRINTEMPS 2023

/*
  ReferenceS:
  - https://www.dimensionengineering.com/info/arduino
  - https://www.dimensionengineering.com/datasheets/Sabertooth2x25v2.pdf
  dipswitch wizzard
  - https://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm

  Hardware:
  - 1x Arduino Uno
  - 1x PixyCam V2
  - 1x Sabertooth 25 Dual 25A Motor Driver version 2
  - 2x dc motors (using servo library since Sabertooth seem to only accept for R/C or micro controller control)
  - 2x Servo motors

  Features:
  - Move 2 motors
  - Move camera
  The circuit:
  - First motor signal out PWM pin 6 to sabertooth S1
  - Second motor signal out PWM pin 5 to sabertooth S2

  Authors:
  SALGUEIRO Alexandre, CORREIA André, AUZARY Frédéric

  sabertooth switch:
  1= down rc mode input
  2= up rc mode input
  3= up using Nimh battery
  4= up mixed mode differential drive
  5= up linear response  ////down exponential response -> troquei pra up, sem exponencial, 31/03
  6= down 4x sensivity mode
*/

#include <Servo.h>
#include <Pixy2.h>
#include <MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <math.h>

#define RADIUS 665  //mm


//motors speed control
float RightVal = 115;  //value to control speed for POT X
float LeftVal = 115;   //value to control speed for POT Y
float cmd = 115;       // base speed of the robot after accelerometer
float cmd0 = 115;      //base speed before accelerometer


// Compte tours
float inc_right;
float inc_left;
float nbtr_right;
float nbtr_left;
float reduction_codeur = 18.0 / 60.0;

//servos camera
float AngleCameraHor = 90;   //standard 90
float AngleCameraVert = 10;  //standard 30

//Camera constants and Target variables
float x_centre = 158; //center of the camera
float y_centre = 104; //center of the camera y
float x_interval = 0.0;

//target
float x_target_detected = 158;
float y_target_detected = 104;
float StandardArea = 3000;
float TargetHeight = 0.1;
float TargetWidth = 0.1;
float TargetArea = 0.1;
float dist_to_target;
bool target_detected;

//obstacle
float x_obstacle_detected = 158;
float y_obstacle_detected = 104;
float ObstacleHeight = 0.1;
float ObstacleWidth = 0.1;
float ObstacleArea = 0.1;
float dist_to_obstacle;
bool obstacle_detected;

float DistanceToTarget = 0;
float SignatureDetected = 0;
float TargetSignature = 1.0;
float consigne = 20000;
float dx;

//Asservissement de la luminosité
int darkness;
int brightness_av;
int darknessMin = 50;
int darknessMax = 200;
int luminositeVar = 10;

//Variables d'asservissement
//float kp = 0.01;
float kpt = 0.3;      //proportional for angle adjustment of the robot (change wheel speed)
float kAngle = 0.03;  //proportional for angle adjustment of the camera angle
//float kstable = 50;
float kluminosite = 5.6;  //convertion factor for luminosity
float KPdeceleration = 0.5;
float KIdeceleration = 0.001;

//variables algorithme champ potentiel
float angle_to_target_mesured;
float angle_to_target_expected;
float angle_to_obstacle;
float Kconv = 90.0 / 158.0; //conversion x screen to angle

// Variiables d'acelerometre
int16_t ax, ay, az;
int16_t gx, gy, gz;
int val;
int prevVal;
int valax;
int valay;
int valaz;
int sumValax = 0.0;

Servo ST1, ST2;             //two motors
Servo rotatHor, rotatVert;  //two servos for the camera
Pixy2 pixy;
MPU6050 mpu;

//variables du champ potentiel
float derivative;
float attraction;
float num;
float den;

void setup() {

  // Comptes tours
  //Voie TargetArea
  pinMode(2, INPUT);
  pinMode(7, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), traitevoieA, CHANGE);

  //Voie B
  pinMode(3, INPUT);
  pinMode(8, INPUT);
  attachInterrupt(digitalPinToInterrupt(3), traitevoieB, CHANGE);

  // attach Servo for the wheel motors and assign pin with min/max
  ST1.attach(6, 1000, 2000);
  ST2.attach(5, 1000, 2000);

  //Servos for the camera orientation
  pinMode(9, OUTPUT);   //Horizontal
  pinMode(10, OUTPUT);  //Vertical
  rotatHor.attach(9);
  rotatVert.attach(10);
  rotatHor.write(90);
  rotatVert.write(30);

  // light sensor
  pinMode(A0, INPUT);
  brightness_av = 255 - kluminosite * analogRead(A0);

  // Inizialize Serial
  Serial.begin(115200);
  pixy.init();

  // Acelerometre
  Wire.begin();
  mpu.initialize();

}


void loop() {
  pixy.setLamp(1, 1); //pixy lamp, not exactly useful

  ST1.write(RightVal);  // sets the motor speed according to the scaled value
  ST2.write(LeftVal);
  nbtr_right = inc_right / (2048) * reduction_codeur; //encoder increments to rotations
  nbtr_left = inc_left / (2048) * reduction_codeur;

  //calcul de la luminosité optimale
  darkness = 255 - kluminosite * analogRead(A0);
  if (darkness < darknessMin) {
    darkness = darknessMin;
  } else if (darkness > darknessMax) {
    darkness = darknessMax;
  }

  //limitation des variations de la luminosité
  if (abs(darkness - brightness_av) < luminositeVar) {
    pixy.setCameraBrightness(darkness);
    brightness_av = darkness;
  }
  accelerometer();

  pixy.ccc.getBlocks(); //importation des variables de la cible et de l'obstacle
  if (pixy.ccc.numBlocks > 1) {
    int i;
    for (i = 0; i < pixy.ccc.numBlocks; i++) {
      SignatureDetected = pixy.ccc.blocks[i].m_signature;

      if (SignatureDetected == 1) {
        x_target_detected = pixy.ccc.blocks[i].m_x;
        y_target_detected = pixy.ccc.blocks[i].m_y;
        TargetHeight = pixy.ccc.blocks[i].m_height;
        TargetWidth = pixy.ccc.blocks[i].m_width;
        TargetArea = TargetHeight * TargetWidth;

        servomotors();

      } else if (SignatureDetected == 4) {
        x_obstacle_detected = pixy.ccc.blocks[i].m_x;
        y_obstacle_detected = pixy.ccc.blocks[i].m_y;
        ObstacleHeight = pixy.ccc.blocks[i].m_height;
        ObstacleWidth = pixy.ccc.blocks[i].m_width;
        ObstacleArea = ObstacleHeight * ObstacleWidth;
      }
    }
    potentialfield();


  } else if (pixy.ccc.numBlocks = 1) {
    SignatureDetected = pixy.ccc.blocks[0].m_signature;

    if (SignatureDetected == 1) {
      x_target_detected = pixy.ccc.blocks[0].m_x;
      y_target_detected = pixy.ccc.blocks[0].m_y;
      TargetHeight = pixy.ccc.blocks[0].m_height;
      TargetWidth = pixy.ccc.blocks[0].m_width;
      TargetArea = TargetHeight * TargetWidth;

      servomotors();
      dx = AngleCameraHor - 90;
      commandmotors();

    } else {
      RightVal = 90;
      LeftVal = 90;
      rotatVert.write(30);
      rotatHor.write(90);
    }
  }
  else {
    servomotors();
    RightVal = 90;
    LeftVal = 90;
  }
}

//Read accelerometer and change CMD pour équilibrer le robot
void accelerometer() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  valax = map(ax, -17000, 17000, 0, 255);
  if (abs(valax - 110) > 15) {
    cmd = cmd0 + (valax - 127) * KPdeceleration;
    if (cmd > (cmd0 + 15)) {
      cmd = cmd0 + 15;
    } else if (cmd < (cmd0 - 5)) {
      cmd = cmd0 - 5;
    }
  }
}

//POSITION DE LA CAMÉRA VERS LA CIBLE
void servomotors() {
  //Direction de la caméra de haut en bas
  if (abs(y_target_detected - y_centre) > 20) {
    int AngleCorrectionVert = abs(y_centre - y_target_detected) * kAngle;
    if (y_target_detected > y_centre) {
      AngleCameraVert += AngleCorrectionVert;
    }
    if (y_target_detected < y_centre) {
      AngleCameraVert -= AngleCorrectionVert;
    }
  }

  //Direction de la caméra de droite à gauche
  if (abs(x_target_detected - x_centre) > 20) {
    int AngleCorrectionHor = abs(x_centre - x_target_detected) * kAngle * 0.5;
    if (x_target_detected < x_centre) {
      if (AngleCorrectionHor == 0) {
        AngleCameraHor += 1;
      }
      AngleCameraHor += AngleCorrectionHor;
    }
    if (x_target_detected > x_centre) {
      if (AngleCorrectionHor == 0) {
        AngleCameraHor -= 1;
      }
      AngleCameraHor -= AngleCorrectionHor;
    }
  }

  //Limitation des Angle de rotation des servomoteurs
  // En hauteur
  if (AngleCameraVert < 10) {
    AngleCameraVert = 10;
  }
  if (AngleCameraVert > 88) {
    AngleCameraVert = 88;
  }
  rotatHor.write(AngleCameraHor);

  // En largeur
  if (AngleCameraHor < 20) {
    AngleCameraHor = 20;
  }
  if (AngleCameraHor > 160) {
    AngleCameraHor = 160;
  }
  rotatVert.write(AngleCameraVert);
}

//FONCTION POUR MOVIMENTATION DES MOTEURS
void commandmotors() {
  if (TargetArea < 16000) {
    if (abs(dx) > x_interval) {
      RightVal = cmd + ((dx / 2) * kpt);
      LeftVal = cmd - ((dx / 2) * kpt);
    }
  }
  else {
    RightVal = 90; //stops motors
    LeftVal = 90;
    rotatVert.write(30);
    rotatHor.write(90);
  }
}

//CALCUL DE LA RÉPULSION/ATTRACTION POUR CONTOURNER L'OBSTACLE
void potentialfield() {
  dist_to_target = pow(StandardArea / TargetArea, 0.5);
  dist_to_obstacle = pow(StandardArea / ObstacleArea, 0.5);

  //soit l'eqution de la forme f(x) = a*exp(-h*dist^2) ----> f'(x) = -2*a*h*dist*exp(-h*dist^2)
  float a = 5; //paramètres pour la forme de la gaussienne
  float h = 0.6;
  angle_to_target_mesured = AngleCameraHor - 90;
  angle_to_obstacle = AngleCameraHor - 90 + (x_obstacle_detected - x_centre) * Kconv;
  derivative = 2 * a * h * dist_to_obstacle * exp(-h * pow(dist_to_obstacle, 2)); //force de répulsion par l'obstacle
  attraction = 1.2; //force d'attraction par la cible
  den = attraction - derivative * cos(angle_to_obstacle * PI / 180.0); //trigonometrie
  num = derivative * sin(angle_to_obstacle * PI / 180.0);
  angle_to_target_expected = -atan2(num , den) * 180.0 / PI  ;
  dx = angle_to_target_mesured - angle_to_target_expected; //direction instantanée désirée
  commandmotors();
}

//ENCODERS
void traitevoieA() {
  if (digitalRead(2) == digitalRead(7)) {
    inc_right++;
  } else {
    inc_right--;
  }
}
void traitevoieB() {
  if (digitalRead(3) == digitalRead(8)) {
    inc_left++;
  } else {
    inc_left--;
  }
}
