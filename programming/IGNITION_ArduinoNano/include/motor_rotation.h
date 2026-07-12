// #ifndef MOTOR_ROTATION_H
// #define  MOTOR_ROTATION_H
// #include<Arduino.h>

// #define PWMA 5
// #define PWMB 6
// #define AIN1 7
// #define AIN2 8
// #define BIN1 9
// #define BIN2 10
// #define STBY_PIN 4

// inline void motorInit(){
//     pinMode(PWMA,OUTPUT);
//     pinMode(PWMB,OUTPUT);

//     pinMode(AIN1,OUTPUT);
//      pinMode(AIN2,OUTPUT);
//       pinMode(BIN1,OUTPUT);
//        pinMode(BIN2,OUTPUT);
    
//     analogWrite(PWMA,0);
//     analogWrite(PWMB,0);
//     digitalWrite(STBY_PIN,HIGH);


// }
// inline void motorForward(uint8_t speed){
//     digitalWrite(AIN1,1);
//      digitalWrite(AIN2,0);
//       digitalWrite(BIN1,1);
//        digitalWrite(BIN2,0);
//     analogWrite(PWMA,speed);
//     analogWrite(PWMB,speed);
// }
// inline void motorBackward(uint8_t speed){
//     digitalWrite(AIN1,0);
//      digitalWrite(AIN2,1);
//       digitalWrite(BIN1,0);
//        digitalWrite(BIN2,1);
//     analogWrite(PWMA,speed);
//     analogWrite(PWMB,speed);
// }
// inline void motorLeft(uint8_t speed){
//     digitalWrite(AIN1,0);
//      digitalWrite(AIN2,1);
//       digitalWrite(BIN1,1);
//        digitalWrite(BIN2,0);
//     analogWrite(PWMA,speed);
//     analogWrite(PWMB,speed);
// }
// inline void motorRight(uint8_t speed){
//     digitalWrite(AIN1,1);
//      digitalWrite(AIN2,0);
//       digitalWrite(BIN1,0);
//        digitalWrite(BIN2,1);
//     analogWrite(PWMA,speed);
//     analogWrite(PWMB,speed);
// }

// inline void motorStop(){
//   analogWrite(PWMA,0);
//   analogWrite(PWMB,0);
//     digitalWrite(AIN1,0);
//      digitalWrite(AIN2,0);
//       digitalWrite(BIN1,0);
//        digitalWrite(BIN2,0);
// }
// #endif
