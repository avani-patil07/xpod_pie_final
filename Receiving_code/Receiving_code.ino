#include <Servo.h>

// J1, J2, J3 pins for each leg
const int LEG_PINS[6][3] = {
  {0, 0, 0},     // leg 1 middle right
  {0, 0, 0},  // leg 2 front right
  {0, 0, 0},  // leg 3 front left
  {0, 0, 0},  // leg 4 middle left
  {0, 0, 0},  // leg 5 back left
  {0, 0, 0},  // leg 6 back right
};

Servo servos[6][3];

void setup() {
  Serial.begin(115200);
  for (int leg = 0; leg < 6; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      if (LEG_PINS[leg][joint] >= 0) servos[leg][joint].attach(LEG_PINS[leg][joint]);
    }
  }
  Serial.println("READY");
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  int start = 0;
  while (start < (int)line.length()) {
    int end = line.indexOf(';', start);
    if (end == -1) end = line.length();
    ApplyLeg(line.substring(start, end));
    start = end + 1;
  }
}

// Moves one leg
void ApplyLeg(String piece) {
  if (!piece.startsWith("L")) return;
  int colon = piece.indexOf(':');
  if (colon == -1) return;
  int comma1 = piece.indexOf(',', colon + 1);
  if (comma1 == -1) return;
  int comma2 = piece.indexOf(',', comma1 + 1);
  if (comma2 == -1) return;

  int leg = piece.substring(1, colon).toInt();
  if (leg < 1 || leg > 6) return;

  float angles[3] = {
    piece.substring(colon + 1, comma1).toFloat(),
    piece.substring(comma1 + 1, comma2).toFloat(),
    piece.substring(comma2 + 1).toFloat()
  };

  for (int joint = 0; joint < 3; joint++) {
    if (LEG_PINS[leg - 1][joint] >= 0) WriteAngle(servos[leg - 1][joint], angles[joint]);
  }
}

// Fractional degrees with microseconds 
void WriteAngle(Servo &s, float angle) {
  angle = constrain(angle, 0, 180);
  s.writeMicroseconds(544 + angle * (2400 - 544) / 180.0);
}