// #ifndef PIDCONTROLLER_H
// #define PIDCONTROLLER_H
// #include<Arduino.h>


// class PIDcontroller{
//     private: 
//         double Kp;
//         double Ki;
//         double Kd;

//         double integral =0.0 ;
//         double prev_error = 0.0 ; 
//         bool first_run = true ;  

//     public: 
//         PIDcontroller(double Kp ,double Ki , double Kd){
//             this->Kp=Kp;
//             this->Ki=Ki;
//             this->Kd=Kd;
//         }
//         double calculate(double setpoint , double measurement , double dt ) {

//              if( dt<=0.0 ) {
//                 return dt = 0.0 ; 
//              }
//              double error = setpoint - measurement ; 
//              double  P = Kp * error ; 
//              integral += error * dt; 
//              double I = Ki*integral ; 

//              double derivative = 0.0 ; 
//              if(!first_run){
//                 derivative =(error-prev_error)/dt; 

//              }
//              else{
//                 first_run= false ; 
//              }
//              double D = Kd*derivative; 

//              double Output = P+ I + D; 

//              prev_error= error;
             

//              return Output ; 


//         }

//         void reset() {
//             integral= 0.0 ; 
//             prev_error= 0.0 ; 
//             first_run= true ;  
//         }

//     };














// #endif