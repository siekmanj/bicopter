/*
 * Jonah Siekmann
 * Written 9/16/2017
 * Bicopter flight controller
 * 
 * Receiver interrupt and PID code loosely based on Joop Brokking's YMFC3D code.
 * Feel free to use or modify as long as you credit me and Joop.
 */

#include <Wire.h>

const int SERVO_RIGHT_OFFSET = 90;   //Servo offset for right servo
const int SERVO_LEFT_OFFSET = 70;     //Servo offset for left servo
const int MPU_ADDR = 0x68;            //i2c address of IMU
const int GYRO_ACCURACY =  1500;      //Number of readings averaged to calculate gyro drift - the higher the better, but the longer the setup time.
const int MAX_PITCH_OUTPUT = 500;
const int MAX_ROLL_OUTPUT = 400;
const int MAX_YAW_OUTPUT = 150;
const int SERVO_REFRESH_HZ = 250/50;

const boolean SPIN_MOTORS = true;    //Safety feature for testing without blades spinning.
const boolean USE_SERVOS = true;     //Debugging feature for ignoring servos

const float KPp = 0.3860;
const float KIp = 0.00000;
const float KDp = 6.5;

const float KPr = .2150; //11
const float KIr = 0.0;  //0.0015
const float KDr = 4; //6.7
  
const float KPy = 6;
const float KIy = 0.0;
const float KDy = 2.0;

float gY, gX, gZ, gYdrift, gXdrift, gZdrift;
float aY, aX, aZ, aTotal, temperature;
float gyroPitchEstimate, gyroRollEstimate, gyroYawEstimate;
float accPitchEstimate, accRollEstimate, accYawEstimate;
float pitchEstimate, rollEstimate, yawEstimate;
float outputPitch, outputRoll, outputYaw, outputThrottle;
float setPointPitch, setPointRoll, setPointYaw, throttle;
float errorTemp, prevErrorP, prevErrorR, prevErrorY, integralP, integralR, integralY, derivativeP, derivativeR, derivativeY, integralT, derivativeT;
float convert_to_degrees = 0;

unsigned long loop_timer, zero_timer, servoRightTimer, servoLeftTimer, esc1Timer, esc2Timer;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, timer_6, current_time, esc_loop_timer;

int old_servoRight = 0;
int old_servoLeft = 0;
int servoRight = 0;
int servoLeft = 0;
int esc1, esc2;

int servoUpdateTick = 0;

int receiver_ch1, receiver_ch2, receiver_ch3, receiver_ch4, receiver_ch5, receiver_ch6;

byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6;

boolean centerGyro = true;

void setup() {
  Serial.begin(230400);
  while(!Serial);
  
  Serial.println("init\nRegister setup...");
  delay(500);
  DDRD |= B11110000;                                 //Set ports 4, 5, 6 and 7 to output
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCICR |= (1 << PCIE1);                             // set PCIE1 to enable PCMSK1 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9) to trigger an interrupt
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10) to trigger an interrupt
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11) to trigger an interrupt
  PCMSK0 |= (1 << PCINT4);                           // set PCINT4 (digital input 12) to trigger an interrupt
  
  Serial.println("Register setup done.");
  delay(500);
 
  gYdrift = 0;
  gXdrift = 0;
  gZdrift = 0;
  gY = 0;
  gX = 0;
  gZ = 0;
  Serial.println("Gyro setup...");
  delay(500);
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send turn-on command
  Wire.write(0x00);                                                    //Send reset command
  Wire.endTransmission();                                              //End the transmission 
  delay(200);
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send accelerometer self-test command
  Wire.write(0x10);                                                    
  Wire.endTransmission();                                              //End the transmission
  delay(200);
  
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send gyroscope self-test command
  Wire.write(0x08);                                                    
  Wire.endTransmission();  
  delay(200);
  Serial.println("Gyro setup done.");
  
  Serial.print("Calibrating gyro");
  
  for(int i = 0; i < GYRO_ACCURACY; i++){
    unsigned long loop_start_t = micros();
    read_MPU6050();
    gYdrift += gY;
    gXdrift += gX;
    gZdrift += gZ;
    if(i % 100 == 0) Serial.print(".");
    while(loop_start_t + 4000 > micros());
  }
  
  Serial.println();
  
  gYdrift /= GYRO_ACCURACY;
  gXdrift /= GYRO_ACCURACY;
  gZdrift /= GYRO_ACCURACY;

  while(receiver_ch3 < 800){
    delay(50);
    Serial.println("Waiting for receiver...");
  }

  while(receiver_ch3 > 1040 || receiver_ch4 > 1040){
     delay(50);
     Serial.println("Waiting for arm signal...");
  }
  
   //Do a swivel to check full range of motion of servos
  if(USE_SERVOS){
    for(int i = 1500; i < 2200; i+=4){
      unsigned long loop_start_t = micros();
      PORTD |= B11000000;
      while(loop_start_t + i > micros());
      PORTD &= B00000000;
      while(loop_start_t + 5000 > micros());
    }
    for(int i = 2000; i > 800; i-=4){
      unsigned long loop_start_t = micros();
      PORTD |= B11000000;
      while(loop_start_t + i > micros());
      PORTD &= B00000000;
      while(loop_start_t + 5000 > micros());
    }
    for(int i = 1000; i < 1500; i+=4){
      unsigned long loop_start_t = micros();
      PORTD |= B11000000;
      while(loop_start_t + i > micros());
      PORTD &= B00000000;
      while(loop_start_t + 5000 > micros());
    }
    for(int i = 0; i < 25; i++){
      unsigned long loop_start_t = micros();
      PORTD |= B11000000;
      while(loop_start_t + 1500 > micros());
      PORTD &= B00000000;
      while(loop_start_t + 5000 > micros());
    }
  }
  
  //Calibrate ESCs
  if(SPIN_MOTORS){
    for(int i = 2000; i > 1000; i-=4){
      unsigned long loop_start_t = micros();
      PORTD |= B00110000;
      delayMicroseconds(i);
      PORTD &= B00000000;
      while(loop_start_t + 4000 > micros());
    }
  }
  
  pitchEstimate = 0;
  rollEstimate = 0;
  yawEstimate = 0;
  gyroRollEstimate = 0;
  gyroPitchEstimate = 0;

  zero_timer = micros();
}

void loop() {
    loop_timer = millis();

    /*
     * Gyro & IMU calculations
     */
    read_MPU6050();

    gY -= gYdrift;
    gX -= gXdrift;
    gZ -= gZdrift;  
    
    gyroPitchEstimate += gY * convert_to_degrees;
    gyroRollEstimate += gX * convert_to_degrees;
    
    gyroPitchEstimate -= gyroRollEstimate * sin(gZ * (convert_to_degrees * (3.142/180)));
    gyroRollEstimate += gyroPitchEstimate * sin(gZ * (convert_to_degrees * (3.142/180))); 

    aTotal = sqrt((aX*aX)+(aY*aY)+(aZ*aZ));            
    accPitchEstimate = -asin((float)aX/aTotal) * 57.2958 - 6; 
    accRollEstimate = -asin((float)aY/aTotal) * -57.2958 + 3; 
    if(centerGyro){ //This is the first loop, and the quadcopter may not be on level ground. So we need to use the accelerometer to center the gyroscope.

      centerGyro = false;
      gyroPitchEstimate = accPitchEstimate;
      gyroRollEstimate = accRollEstimate;
      
    }else{
      
      gyroPitchEstimate = gyroPitchEstimate * 0.9996 + accPitchEstimate * 0.0004;
      gyroRollEstimate = gyroRollEstimate * 0.9996 + accRollEstimate * 0.0004;

    }

    pitchEstimate = pitchEstimate * 0.7 + gyroPitchEstimate * 0.3;
    rollEstimate = rollEstimate * 0.7 + gyroRollEstimate * 0.3; 

    /*
     * Receiver input
     */
    if(receiver_ch4 > 1508) setPointYaw = (1508 - receiver_ch4)/10;
    else if(receiver_ch4 < 1492) setPointYaw = (1492 - receiver_ch4)/10;
    
    throttle = receiver_ch3;
    
    if(receiver_ch2 > 1508) setPointPitch = (1508 - receiver_ch2)/1.5;
    else if(receiver_ch2 < 1492) setPointPitch = (1492 - receiver_ch2)/1.5;
    
    if(receiver_ch1 > 1508) setPointRoll = (receiver_ch1 - 1508)/-1.6;
    else if(receiver_ch1 < 1492) setPointRoll = (receiver_ch1 - 1492)/-1.6;

    /*
     * PID calculations
     */
    calculate_pid();

    /*
     * ESC & Servo PWM output calculations
     */
    
    esc1 = throttle + outputRoll;
    esc2 = throttle - outputRoll;

    if(esc1 < 1000 || throttle < 1020) esc1 = 1000;
    if(esc2 < 1000 || throttle < 1020) esc2 = 1000;

    /*
     * ESC & Servo PWM output generation
     */
    boolean moveServos = false;
    
    if(servoUpdateTick == SERVO_REFRESH_HZ){
      servoUpdateTick = 0;

      old_servoRight = servoRight;
      old_servoLeft = servoLeft;
      servoRight = 1500 + SERVO_RIGHT_OFFSET + outputPitch + outputYaw;
      servoLeft = 1500 + SERVO_LEFT_OFFSET - outputPitch + outputYaw;
      //moveServos = true;
      
      if(old_servoRight - servoRight > 0 || old_servoRight - servoRight < 0){ //Digital servos can only handle so many PWM signals per second. If we do one every 8 loops thats every 32 ms - roughly 31hz. Google says 40hz is upper bound, so we gud.
        moveServos = true;
      }else{
        servoRight = old_servoRight;
      }
      if(old_servoLeft - servoLeft > 0 || old_servoLeft - servoLeft < 0){
        moveServos = true;
      }else{
        servoLeft = old_servoLeft;
      }
    }else{
      servoUpdateTick++;
    }
    //Serial.print(rollEstimate); Serial.print(", "); Serial.println(accRollEstimate);
    while(zero_timer + 4000 > micros());
    zero_timer = micros();

    esc1Timer = zero_timer + esc1;
    esc2Timer = zero_timer + esc2;
    servoRightTimer = zero_timer + servoRight;
    servoLeftTimer = zero_timer + servoLeft;
    
    if(SPIN_MOTORS){
      PORTD |= B00110000;
    }
    if(moveServos && USE_SERVOS){
      PORTD |= B11000000;
    }
    while(PORTD >= 16){
      esc_loop_timer = micros();
      if(servoRightTimer <= esc_loop_timer) PORTD &= B01111111;
      if(servoLeftTimer <= esc_loop_timer) PORTD &= B10111111;
      if(esc1Timer <= esc_loop_timer) PORTD &= B11011111;
      if(esc2Timer <= esc_loop_timer) PORTD &= B11101111;
    }

    convert_to_degrees = 1/(1000/(millis()-loop_timer)*65.5);
}

void read_MPU6050(){
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the address of ACCEL_XOUT (first register we want to read)
  Wire.endTransmission();                                              //End transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes
  
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  
  aX = Wire.read() << 8 | Wire.read();                                  
  aY = Wire.read( )<< 8 | Wire.read();                                  
  aZ = Wire.read() << 8 | Wire.read();                                  
  temperature = Wire.read() << 8 | Wire.read();                         
  gX = Wire.read() << 8 | Wire.read();                                 
  gY = Wire.read() << 8 | Wire.read();                                 
  gZ = Wire.read() << 8 | Wire.read();

                                   
}

void calculate_pid(){

  errorTemp = setPointPitch - (pitchEstimate * 17); //This is the proportional term, which is fairly basic.
  
  integralP += errorTemp; //The integral is just the area under the curve, so all we have to do is keep adding the errors together.
  derivativeP = (errorTemp - prevErrorP); //calculate the rate of change based on this loop's error and last loop's error.
  prevErrorP = errorTemp;
  outputPitch = KPp*errorTemp + KIp*integralP + KDp*derivativeP;
  
  if(outputPitch > MAX_PITCH_OUTPUT) outputPitch = MAX_PITCH_OUTPUT; //make sure the output stays within a certain range.
  if(outputPitch < -MAX_PITCH_OUTPUT) outputPitch = -MAX_PITCH_OUTPUT;

  
  errorTemp = setPointRoll - (rollEstimate * 17);
  integralR += errorTemp;
  derivativeR = (errorTemp - prevErrorR);
  prevErrorR = errorTemp;
  outputRoll = KPr*errorTemp + KIr*integralR + KDr*derivativeR;
  if(outputRoll > MAX_ROLL_OUTPUT) outputRoll = MAX_ROLL_OUTPUT;
  if(outputRoll < -MAX_ROLL_OUTPUT) outputRoll = -MAX_ROLL_OUTPUT;
  
  
  errorTemp = (setPointYaw) - gZ * convert_to_degrees * 17;
  integralY += errorTemp;
  derivativeR = (errorTemp - prevErrorY);
  prevErrorY = errorTemp;
  outputYaw = KPy*errorTemp + KIy*integralY + KDy*derivativeY;
  if(outputYaw > MAX_YAW_OUTPUT) outputYaw = MAX_YAW_OUTPUT;
  if(outputYaw < -MAX_YAW_OUTPUT) outputYaw = -MAX_YAW_OUTPUT;


  if(throttle < 1020){
    errorTemp = 0;
    prevErrorY = 0;
    outputYaw = 0;
    outputPitch = -1 * pitchEstimate * 16;
    outputRoll = 0;
    integralP = 0;
    integralR = 0;
    integralY = 0;
    prevErrorP = 0;
    prevErrorY = 0;
    prevErrorR = 0;
  }
  
}



ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_ch1 = current_time - timer_1;         //Channel 1 is current_time - timer_1
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_ch2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_ch3 = current_time - timer_3;         //Channel 3 is current_time - timer_3
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_ch4 = current_time - timer_4;         //Channel 4 is current_time - timer_4
  }


  //Channel 5=========================================
  if(PINB & B00010000 ){                                       //Is input 12 high?
    if(last_channel_5 == 0){                                   //Input 12 changed from 0 to 1
      last_channel_5 = 1;                                      //Remember current input state
      timer_5 = current_time;                                  //Set timer_5 to current_time
    }
  }
  else if(last_channel_5 == 1){                                //Input 12 is not high and changed from 1 to 0
    last_channel_5 = 0;                                        //Remember current input state
    receiver_ch5 = current_time - timer_5;         //Channel 4 is current_time - timer_4
  }
  //Channel 6=========================================
  if(PINB & B00100000 ){                                       //Is input 13 high?
    if(last_channel_6 == 0){                                   //Input 13 changed from 0 to 1
      last_channel_6 = 1;                                      //Remember current input state
      timer_6 = current_time;                                  //Set timer_6 to current_time
    }
  }
  else if(last_channel_6 == 1){                                //Input 12 is not high and changed from 1 to 0
    last_channel_6 = 0;                                        //Remember current input state
    receiver_ch6 = current_time - timer_6;        
  }
}
