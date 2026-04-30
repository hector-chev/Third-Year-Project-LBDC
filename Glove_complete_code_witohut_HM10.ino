//INCLUDE LIBRARIES -------------------------------------------------------------------------------

#include<Wire.h>

// ------------------------------------------------------------------------------INLCUDE LIBRARIES

//PINS-------------------------------------------------------------------------------------------

const int glove_led_RED = 6;
const int glove_led_YELLOW = 5;
const int glove_led_GREEN = 4;
const int sensorPin = A0;

//-----------------------------------------------------------------------------------------PINS

//CONSTANTS --------------------------------------------------------------------------------------

const int iterations = 300; // iterations for the calibration
const int MPU6050_addr=0x68; // MPU I2C adress
const float K_fusion = 0.94;
const unsigned int N = 100;

// -------------------------------------------------------------------------------------CONSTANTS

//GLOBAL VARIABLES -------------------------------------------------------------------------------

int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;
double C1_X = 0, C1_Y = 0, C2_X = 0, C2_Y = 0; //calibration offsets
double phi_rad, theta_rad, phi_trig, theta_trig; //angles calculated using trigonometry
double theta_temp = 0, theta_integral, phi_temp = 0, phi_integral, theta, phi;
unsigned long int global_dt, global_dt_prev;
unsigned int ave_counter = 0;
double phi_sum = 0, theta_sum = 0;


// ------------------------------------------------------------------------------GLOBAL VARIABLES

void setup() {
  Serial.begin(115200);
  pinMode(glove_led_RED, OUTPUT);
  pinMode(glove_led_YELLOW, OUTPUT);
  pinMode(glove_led_GREEN, OUTPUT);
  led_null();
  
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  while (calibrate() == 1){
    Serial.println("Calibration Error! - Trying Again, Please Keep Hand Steady");
  }
  global_dt_prev = micros();

}

void loop() {
  load_MPU_values();
  //unsigned long int AccZ_squared = AccZ*AccZ;
  //angle calculation using AccX, AccY and AccZ
  phi_rad = atan(-double(AccX)/sqrt(((double(AccY)*double(AccY))+(double(AccZ)*double(AccZ)))));
  theta_rad = atan(double(AccY)/double(AccZ));
  phi_trig = phi_rad*180/PI;
  theta_trig = theta_rad*180/PI;
  //done angle calc using trig


  global_dt = (micros() - global_dt_prev);
  global_dt_prev = micros();
  theta_temp += ((double(GyroX)/131.0) - C1_X)*double(global_dt)/1000000; //pitch
  phi_temp += ((double(GyroY)/131.0) - C1_Y)*double(global_dt)/1000000; //roll
  theta_integral = theta_temp - C2_X;
  phi_integral = phi_temp - C2_Y;

  theta = K_fusion*theta_integral + (1-K_fusion)*theta_trig;
  phi = K_fusion*phi_integral + (1-K_fusion)*phi_trig;
  // Serial.print("Theta (X): ");
  //Serial.println(theta);
  // Serial.print("Phi (Y): ");
  //Serial.println(AccZ_squared);
  double theta_mapped = theta+90;
  double phi_mapped = phi+90;

  // Constrain just in case
  theta_mapped = constrain(theta_mapped, 0, 180);
  phi_mapped = constrain(phi_mapped, 0, 180);

  // Send over serial in easy-to-parse format

  

  if (ave_counter < N){
    theta_sum += theta_mapped;
    phi_sum += phi_mapped;
    ave_counter++;
  }
  else{
    Serial.print("T:");
    Serial.print(theta_sum/N);
    Serial.print(",P:");
    Serial.println(phi_sum/N);
    theta_sum = 0;
    phi_sum = 0;
    ave_counter = 0;
    led_transmitting();
  }

  // if ((float(analogRead(sensorPin))*(5.0/1024.0)*5/3.3) < 2){
  //   led_error();
  // }
  // else if ((float(analogRead(sensorPin))*(5.0/1024.0)*5/3.3) < 3.5){
  //   led_calibrating();
  // }
  // else{
  //   led_transmitting();
  // }


  delay(5); 
}

//I2C -----------------------------------------------------------------------------------------------

void load_MPU_values(){
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  AccX=Wire.read()<<8|Wire.read();
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();
}

// ----------------------------------------------------------------------------------------------I2C

//CALIBRATION ---------------------------------------------------------------------------------------

int calibrate(){
  led_calibrating(); // set led to yellow
  bool calibration_error_flag = 0;
  bool calibration_completion_flag = 0;
  bool j_zero_flag = 0;
  int i = 0, j = 0; //initialise counters, i = C1 term, j = C2 term
  double sum_GyroX = 0, sum_GyroY = 0, sum_GyroX_integral = 0, sum_GyroY_integral = 0, average_GyroX_raw = 0, average_GyroY_raw = 0;
  unsigned long int cal_dt, cal_dt_prev;
  while (calibration_completion_flag == 0){
    if ((i < iterations) && (j == 0)){
      load_MPU_values();
      sum_GyroX += GyroX; sum_GyroY += GyroY; // take the sum over n=iterations
      i++;
      //Serial.println(i); //GET RID OF THIS ##############################################################
    }
    else if ((i >= iterations) && (j == 0) && (j_zero_flag == 0)){
      average_GyroX_raw = sum_GyroX/iterations; average_GyroY_raw = sum_GyroY/iterations; //average of GyroX&Y
      C1_X = average_GyroX_raw/131.0; C1_Y = average_GyroY_raw/131.0; // convert from raw to C1_X&Y
      cal_dt_prev = micros();
      j_zero_flag = 1;
    }
    else if ((i >= iterations) && (j < iterations)){
      load_MPU_values();
      cal_dt = micros() - cal_dt_prev;
      cal_dt_prev = micros();
      //Serial.println(((float(GyroX)/131.0) - C1_X)*float(cal_dt)/1000000, 10);
      sum_GyroX_integral += ((float(GyroX)/131.0) - C1_X)*float(cal_dt)/1000000; // do the integral with C1_X cal
      sum_GyroY_integral += ((float(GyroY)/131.0) - C1_Y)*float(cal_dt)/1000000; // do the integral with C1_Y cal
      j++;
      //Serial.println(j); //GET RID OF THIS ##############################################################
    }
    else{
      C2_X = sum_GyroX_integral; C2_Y = sum_GyroY_integral;
      // Serial.print("C1_X: ");
      // Serial.println(C1_X);
      // Serial.print("C2_X: ");
      // Serial.println(C2_X);
      // Serial.print("C1_Y: ");
      // Serial.println(C1_Y);
      // Serial.print("C2_Y: ");
      // Serial.println(C2_Y);
      sum_GyroX = 0; sum_GyroY = 0; sum_GyroX_integral = 0; sum_GyroY_integral = 0; j = 0; i = 0; j_zero_flag = 0; 
      calibration_completion_flag = 1;
    }
  }

  if (calibration_error_flag == 1){
    //led_error(); // display red led if error occurs
  }
  else{
    //Serial.println("LED NULL");
    //led_transmitting(); // display green otherwise
  }

  return calibration_error_flag; // return if there was a calibration error

}

// ---------------------------------------------------------------------------------------CALIBRATION

//LED FUNCTIONS -------------------------------------------------------------------------------------

void led_null(){
  digitalWrite(glove_led_RED, LOW);
  digitalWrite(glove_led_YELLOW, LOW);
  digitalWrite(glove_led_GREEN, LOW);
  return;
}

void led_transmitting(){
  digitalWrite(glove_led_RED, LOW);
  digitalWrite(glove_led_YELLOW, LOW);
  digitalWrite(glove_led_GREEN, HIGH);
  return;
}

void led_calibrating(){
  digitalWrite(glove_led_RED, LOW);
  digitalWrite(glove_led_YELLOW, HIGH);
  digitalWrite(glove_led_GREEN, LOW);
  return;
}

void led_error(){
  digitalWrite(glove_led_RED, HIGH);
  digitalWrite(glove_led_YELLOW, LOW);
  digitalWrite(glove_led_GREEN, LOW);
  return;
}

//------------------------------------------------------------------------------------LED FUNCTIONS