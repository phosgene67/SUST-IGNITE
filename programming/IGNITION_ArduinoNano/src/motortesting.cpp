// =============================================================
//  MOTOR TEST - Full speed forward 10s, then backward 10s
//  Uses same pin mapping as line_follower_FINAL.ino
// =============================================================

#include<Arduino.h>
#define PWMA 3      // Right motor PWM (OC2B)
#define AI1 7
#define AI2 6
#define STBY 8
#define PWMB 10     // Left motor PWM (OC1B)
#define BI1 11
#define BI2 12

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);  // take driver out of standby
}

void loop() {
  // ---- FORWARD, full speed, 10s ----
  digitalWrite(AI1, LOW);
  digitalWrite(AI2, HIGH);
  analogWrite(PWMA, 255);

  digitalWrite(BI1, LOW);
  digitalWrite(BI2, HIGH);
  analogWrite(PWMB, 255);

  delay(10000);

  // ---- Brief stop to avoid a hard direction-reversal spike ----
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(200);

  // ---- BACKWARD, full speed, 10s ----
  digitalWrite(AI1, HIGH);
  digitalWrite(AI2, LOW);
  analogWrite(PWMA, 255);

  digitalWrite(BI1, HIGH);
  digitalWrite(BI2, LOW);
  analogWrite(PWMB, 255);

  delay(10000);

  // ---- Stop before looping again ----
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delay(200);
}