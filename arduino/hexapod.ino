#include <Servo.h>
#include <Math.h>

// Debug
#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINTL(x)  Serial.println(x)
  #define DEBUG_PRINT(x)  Serial.print(x)
#else
  #define DEBUG_PRINTL(x)
  #define DEBUG_PRINT(x)
#endif

#define J1Pin 3
#define J2Pin 5
#define J3Pin 6

Servo Joint1;
Servo Joint2;
Servo Joint3;

void setup() {
  Serial.begin(115200);

  Joint1.attach(J1Pin);
  Joint2.attach(J2Pin);
  Joint3.attach(J3Pin);

  Serial.println("Enter X Y Z (e.g., 100 100 100):");
}

void loop() {
  if (Serial.available()) {
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();

    delay(10);
    while (Serial.available()) Serial.read();  // clear buffer
    CartesianMove(x, y, z); // send commands
  }
}

void CartesianMove(float X, float Y, float Z) {
  float theta1, theta2, theta3;
  
  if (!IK(X, Y, Z, &theta1, &theta2, &theta3)) {
    DEBUG_PRINTL("Target unreachable");
  }
  
  Joint1.write(theta1);
  Joint2.write(theta2);
  Joint3.write(theta3);

  DEBUG_PRINT("Theta1 (coxa): "); DEBUG_PRINTL(theta1);
  DEBUG_PRINT("Theta2 (femur): "); DEBUG_PRINTL(theta2);
  DEBUG_PRINT("Theta3 (tibia): "); DEBUG_PRINTL(theta3);
}

void ZeroServos() {
  Joint1.write(0);
  Joint2.write(0);
  Joint3.write(0);
}

