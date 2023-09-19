#include <Pixy2.h>
#include <PIDLoop.h>
#include <Servo.h>

#define X_CENTER (pixy.frameWidth / 2)

Pixy2 pixy;

PIDLoop headingLoop(5000, 0, 0, false);

//servos camera
float AngleCameraHor = 90;   //standard 90
float AngleCameraVert = 10;  //standard 30

//Asservissement de la luminosité
float kluminosite = 5.6;  //convertion factor for luminosity
int darkness;
int brightness_av;
int darknessMin = 50;
int darknessMax = 200;
int luminositeVar = 10;

//Value of vectors
int speed = 150;  //speed ahead
int distance_antes;
int distance_depois;

//Points x and y of vector
uint8_t m_x0;
uint8_t m_y0;
uint8_t m_x1;
uint8_t m_y1;

int value;

//Asservissement vitesse RIGHT
long counterR(0);
float trsR(0);
int KpR = 0.6 * 950;
float consigneR(20);  // 20 consgine is 3 rotations
float errR(0);
float cmdR(0);
float erreur_avR = 0;
float erreur_derR = 0;

//Asservissement vitesse RIGHT
long counterL(0);
float trsL(0);
int KpL = 0.6 * 950;
float consigneL(20);  // 20 consgine is 3 rotations
float errL(0);
float cmdL(0);
float erreur_avL = 0;
float erreur_derL = 0;

bool flag = false;

Servo ST1, ST2;             //two motors
Servo rotatHor, rotatVert;  //two servos for the camera

void setup() {
  Serial.begin(115200);
  Serial.print("Starting...\n");

  pixy.init();
  pixy.setLamp(1, 1);
  pixy.changeProg("line_tracking");

  // light sensor
  pinMode(A0, INPUT);
  brightness_av = 255 - kluminosite * analogRead(A0);

  // attach Servo for the wheel motors and assign pin with min/max
  ST1.attach(6, 1000, 2000);
  ST2.attach(5, 1000, 2000);

  // Comptes tours
  //Voie TargetArea
  pinMode(2, INPUT);
  pinMode(7, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), interruptEnc1forpin27, CHANGE);
  attachInterrupt(digitalPinToInterrupt(7), interruptEnc2forpin27, CHANGE);

  //Voie B
  pinMode(3, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), interruptEnc1forpin38, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), interruptEnc2forpin38, CHANGE);

  //Servos for the camera orientation
  pinMode(9, OUTPUT);   //Horizontal
  pinMode(10, OUTPUT);  //Vertical
  rotatHor.attach(9);
  rotatVert.attach(10);
  rotatHor.write(90);
  rotatVert.write(80);
}

void loop() {

  int8_t res;
  int32_t error;
  int left, right;
  char buf[96];

  //calcul de la luminosité optimale
  darkness = 255 - kluminosite * analogRead(A0);
  if (darkness < darknessMin) {
    darkness = darknessMin;
  } else if (darkness > darknessMax) {
    darkness = darknessMax;
  }

  //limitation des variations de la luminosité
  if (abs(darkness) - abs(brightness_av) < luminositeVar) {
    pixy.setCameraBrightness(darkness);
    brightness_av = darkness;
  }

  // Get latest data from Pixy, including main vector, new intersections and new barcodes.
  res = pixy.line.getAllFeatures();

  // If error or nothing detected, stop motors
  if (res <= 0) {
    if (!flag) {
      counterR = 0;
      counterL = 0;
      flag = true;
    }
    trsR = counterR / (2 * 2 * 256.0);  // Precision 256 ticks/tr, CHANGE = RISING & FALLING, 2 ways
    trsL = counterL / (2 * 2 * 256.0);  // Precision 256 ticks/tr, CHANGE = RISING & FALLING, 2 ways
    pidControlR();
    pidControlL();
  }

  // We found the vector...
  if (res & LINE_VECTOR) {

    // Calculate heading error with respect to m_x1, which is the far-end of the vector,
    // the part of the vector we're heading toward.
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;  // erro em relaçcao a ponta do vetor (final) (+-39)

    //pixy.line.vectors->print();

    // Perform PID calcs on heading error.
    headingLoop.update(error);

    distance_antes = map(error, -39, 39, -20, 20);

    if (abs(distance_antes - distance_depois) < 1) {  // erro devido a inconsistencia do vetor na pixy
      if (error == 0) {
        right = speed;
        left = speed;
      } else {
        right = right;
        left = left;
      }
    } else {
      if (error > 0) {
        right = speed + abs(distance_antes);
        left = speed - abs(distance_antes);
      } else if (error < 0) {
        right = speed - abs(distance_antes);
        left = speed + abs(distance_antes);
      } else if (error == 0) {
        right = speed;
        left = speed;
      }
    }

    distance_depois = map(error, -39, 39, -20, 20);

    // If vector is heading away from us (arrow pointing up), things are normal.
    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1) {
      // ... but slow down a little if intersection is present, so we don't miss it.
      if (pixy.line.vectors->m_flags & LINE_FLAG_INTERSECTION_PRESENT) {
        //left += ZUMO_SLOW;
        //right += ZUMO_SLOW;
      }
    }
    ST1.write(left);
    ST2.write(right);
    flag = false;
  }

  // If barcode, acknowledge with beep, and set left or right turn accordingly.
  // When calling setNextTurn(), Pixy will "execute" the turn upon the next intersection,
  // making the left or right branch in the intersection the new main vector, depending on
  // the angle passed to setNextTurn(). The robot will then follow the branch.
  // If the turn is not set, Pixy will choose the straight(est) path by default, but
  // the default turn can be changed too by calling setDefaultTurn(). The default turn
  // is normally 0 (straight).
  if (res & LINE_BARCODE) {
    if (pixy.line.barcodes->m_code == 4)
      value = 4;
    else if (pixy.line.barcodes->m_code == 14)
      value = 14;
    else if (pixy.line.barcodes->m_code == 8) {
      for (int t = 0; t < 80; t++) {
        delay(100);
        right = right - t;
        left = left - t;
        ST1.write(left);
        ST2.write(right);
        if (right <= 90 or left <= 90) {
          ST1.write(90);
          ST2.write(90);
        }
      }
      Serial.print("STOP");
      exit(0);
    }
  }

  // If intersection, do nothing (we've already set the turn), but acknowledge with a beep.
  if (res & LINE_INTERSECTION) {
    if (res & LINE_BARCODE) {
      if (pixy.line.barcodes->m_code == 4)
        value = 4;
      else if (pixy.line.barcodes->m_code == 14)
        value = 14;
      else if (pixy.line.barcodes->m_code == 8) {
        for (int t = 0; t < 80; t++) {
          delay(100);
          right = right - t;
          left = left - t;
          ST1.write(left);
          ST2.write(right);
          if (right <= 90 or left <= 90) {
            ST1.write(90);
            ST2.write(90);
          }
        }
        Serial.print("STOP");
        exit(0);
      }
    }

    switch (value) {
      case 4:
        pixy.line.setNextTurn(90);
        ST1.write(90);
        ST2.write(90);
        //for (int t = 0; t < 80; t++) {
          //delay(100);
          //right = right + t;
          //left = left - t;
          //ST1.write(left);
          //ST2.write(right);
        //}
        break;
      case 14:
        pixy.line.setNextTurn(-90); /*
        for (int t = 0; t < 80; t++) {
        delay(100);
        right = right + t;
        left = left - t;
        ST1.write(left);
        ST2.write(right);
        if (right <= 90 or left <= 90) {
          ST1.write(90);
          ST2.write(90);
        }
      }*/
        break;
      default:
        pixy.line.setNextTurn(0);
        //ST1.write(left);
        //ST2.write(right);
        //delay(1000);
        break;
    }
  }
}

void pidControlR() {
  errR = consigneR - trsR;

  int cmdR = errR * KpR;
  erreur_avR = errR;
  int cmd_2R = map(cmdR, 18800, 0, 170, 90);
  rotMotR(cmd_2R);
}

void interruptEnc1forpin38() {
  if (digitalRead(3) == digitalRead(8)) {
    counterR -= 1;
  } else {
    counterR += 1;
  }
}

void interruptEnc2forpin38() {
  if (digitalRead(3) == digitalRead(8)) {
    counterR += 1;
  } else {
    counterR -= 1;
  }
}

void rotMotR(int spdR) {
  if (spdR > 170) {
    spdR = 170;
    int sensR = 1;
  }
  if (spdR < 90) {
    spdR = 90;
  }
  Serial.println(spdR);
  if (spdR < 90) {
    ST2.write(90);
  } else {
    ST2.write(spdR);
  }
}

void pidControlL() {
  errL = consigneL - trsL;

  int cmdL = errL * KpL;
  erreur_avL = errL;
  int cmd_2L = map(cmdL, 18800, 0, 170, 90);
  rotMotL(cmd_2L);
}

void interruptEnc1forpin27() {
  if (digitalRead(2) == digitalRead(7)) {
    counterL += 1;
  } else {
    counterL -= 1;
  }
}

void interruptEnc2forpin27() {
  if (digitalRead(2) == digitalRead(7)) {
    counterL -= 1;
  } else {
    counterL += 1;
  }
}

void rotMotL(int spdL) {
  if (spdL > 170) {
    spdL = 170;
    int sensL = 1;
  }
  if (spdL < 90) {
    spdL = 90;
  }
  Serial.println(spdL);
  if (spdL < 90) {
    ST1.write(90);
  } else {
    ST1.write(spdL);
  }
}
