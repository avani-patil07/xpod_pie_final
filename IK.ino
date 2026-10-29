float L1 = 65;  // coxa length (mm)
float L2 = 134; // femur length (mm)
float L3 = 173; // tibia length (mm)

/*
 * Returns angles relative to the servo (0-180)
 */

bool IK(float x, float y, float z, float* theta1, float* theta2, float* theta3) {
  *theta1 = 90 - atan2(z, x)*180/PI;

  float r_horiz = sqrt(x*x + z*z) - L1;
  float r = sqrt(r_horiz*r_horiz + y*y);

  // Check whether the target is within the leg's reach
  if (r < abs(L2 - L3) || r > L2 + L3) {
    return false;
  }

  float phi2 = atan2(y, r_horiz);
  float phi1 = acos((r*r + L2*L2 - L3*L3) / (2*r*L2));
  float phi3 = acos((L2*L2 + L3*L3 - r*r) / (2*L2*L3));
  
  *theta2 = 90 - (phi1+phi2)*180/PI;
  *theta3 = 180 - phi3*180/PI;

  return true;
}