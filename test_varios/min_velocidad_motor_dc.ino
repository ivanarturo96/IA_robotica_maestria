// Sube el PWM de a 5 cada 600 ms; anota cuando la rueda empieza a girar.
void rampTest(void (*setMotor)(int), int dir) {
  for (int pwm = 30; pwm <= 110; pwm += 5) {
    Serial.println(pwm);
    setMotor(dir * pwm);
    delay(600);
  }
  setMotor(0);
  delay(1500);
}

void loop() {

  rampTest(setLeftMotor, +1); 
  rampTest(setLeftMotor, -1);
  rampTest(setRightMotor, +1); 
  rampTest(setRightMotor, -1);
}