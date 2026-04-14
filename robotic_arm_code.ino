#include <Servo.h>
#include <math.h>
#include <string.h>

//member lengths
const float l1 = 45;
const float l2 = 103;

const int elbowPin = 11;
const int shoulderPin = 10;
const int clawPin = 9;

Servo elbow;
Servo shoulder;
Servo claw;

struct IKResult {
  float theta1;
  float theta2;
  bool  reachable = true;
};

IKResult solveIK(float x, float y) {
  IKResult result;
  float r2 = x * x + y * y;
  float r  = sqrt(r2);

  float rMin = fabs(l1 - l2);
  float rMax = l1 + l2;

  if (r < rMin || r > rMax) {
    result.reachable = false;
    result.theta1 = 0;
    result.theta2 = 0;
    Serial.print("ERROR: distance ");
    Serial.print(r);
    Serial.print("mm is out of reachable range [");
    Serial.print(rMin);
    Serial.print(", ");
    Serial.print(rMax);
    Serial.println("]");
    return result;
  }

  float alph = acos((l1*l1 + l2*l2 - r2) / (2*l1*l2));
  float theta2 = M_PI - alph;
  float gamm = atan2(y, x);
  float beta = atan2(l2*sin(theta2), l1 + l2*cos(theta2));
  float theta1 = gamm - beta;

  result.theta1 = theta1 * 57.2958;
  result.theta2 = theta2 * 57.2958;

  Serial.print("r2=");      Serial.println(r2);
  Serial.print("alph=");    Serial.println(alph);
  Serial.print("theta1=");  Serial.println(result.theta1);
  Serial.print("theta2=");  Serial.println(result.theta2);
  Serial.print("reachable="); Serial.println(
    !(result.theta1 < 0 || result.theta1 > 180 ||
      result.theta2 < 0 || result.theta2 > 180));

  if (result.theta1 < 0 || result.theta1 > 180 ||
      result.theta2 < 0 || result.theta2 > 180) {
    result.reachable = false;
  }

  return result;
}

void writeServos(float theta1_deg, float theta2_deg) {
  int s1 = (int)constrain(theta1_deg, 0, 180);
  int s2 = (int)constrain(theta2_deg, 0, 180);

  shoulder.write(s1);
  elbow.write(s2);

  Serial.print("Servo1 (theta1) = ");
  Serial.print(s1);
  Serial.print("°  |  Servo2 (theta2) = ");
  Serial.print(s2);
  Serial.println("°");
}

void moveTo(float x, float y) {
  Serial.print("Moving to (");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.println(")");

  IKResult ik = solveIK(x, y);
  if (ik.reachable) {
    writeServos(ik.theta1, ik.theta2);
  } else {
    Serial.println("Skipping — target out of workspace.");
  }
}

String inputBuffer = "";
void parseAndMove(String s) {
  s.trim();
  int commaIndex = s.indexOf(',');

  if (commaIndex == -1) {
    Serial.println("Bad format. Use: x,y  (e.g. 120.0,60.5)");
    return;
  }

  float x = s.substring(0, commaIndex).toFloat();
  float y = s.substring(commaIndex + 1).toFloat();

  moveTo(x, y);
}

void setup() {
  Serial.begin(9600);
  elbow.attach(elbowPin);
  shoulder.attach(shoulderPin);
  claw.attach(clawPin);

  shoulder.write(90);
  elbow.write(90);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        parseAndMove(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

