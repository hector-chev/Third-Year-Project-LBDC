#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define PI 3.14159265

float global_variable, global_variable2;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

class Display{
  public:
    void print_Serial(const char* message) {
      Serial.print(message);
    }
    void print_Serial_ln(const char* message) {
      Serial.println(message);
    }
    void print_MPU_0x0(const char* message){
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(message);
      display.display();
    }
    void print_MPU_0x0(double value){
      display.clearDisplay();
      display.setCursor(0,0);
      display.println(value);
      display.display();
    }
    void print_MPU(const char* message, int x, int y){
      display.clearDisplay();
      display.setCursor(x, y);
      display.println(message);
      display.display();
    }
    void print_MPU(double value, int x, int y){
      display.clearDisplay();
      display.setCursor(x, y);
      display.println(value);
      display.display();
    }
    void print_OLED_3_vals(float a, float b, float c){
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(a);
      display.setCursor(0, 20);
      display.println(b);
      display.setCursor(0, 40);
      display.println(c);
      display.display();
    }

    void print_OLED_4_vals(float a, float b, float c, float d){
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println(a);
      display.setCursor(0, 20);
      display.println(b);
      display.setCursor(0, 40);
      display.println(c);
      display.setCursor(40, 30);
      display.println(d);
      display.display();
    }
};

Display display_mine;

class Servo_Safe_SET {
  public:
    Servo_Safe_SET(Servo &servo_in, const char* servo_name,
                   int attachment, float lower_limit,
                   float upper_limit, float base_value)
      : servo(servo_in),
        upper(upper_limit),
        lower(lower_limit),
        base(base_value),
        name(servo_name),
        pin(attachment) {}

    void begin() {
      servo.attach(pin);
      check_and_set(base);
    }

    void check_and_set(double proposed_value) {
      if ((proposed_value < lower) || (proposed_value >= upper)) {
        //display_mine.print_Serial("Error: ");
        //display_mine.print_Serial(name);
        //display_mine.print_Serial_ln(" out of range");
      }

      proposed_value = constrain(proposed_value, lower, upper);
      servo.write(proposed_value);
    }

  private:
    const char* name;
    float upper, lower, base;
    int pin;
    Servo &servo;
};


class Servo_PID {
  public:
    Servo_PID(Servo &servo_in, const char* name,
              int attachment, float lower_limit,
              float upper_limit, float base_value,
              float servo_calibration_m,
              float servo_calibration_c,
              float Kp, float Ki, float Kd)
      : servo_safe(servo_in, name, attachment,
                   lower_limit, upper_limit, base_value),
        m(servo_calibration_m),
        c(servo_calibration_c),
        K_p(Kp), K_i(Ki), K_d(Kd){}

    void begin() {
      servo_safe.begin();
    }

    void update_servo_with_PID(double analog_raw, double desired_val) {

      analog_deg = (analog_raw - c) / m;
      error = desired_val - analog_deg;

      time_now = millis();

      if (time_prev == 0) {
        time_prev = time_now;
        analog_deg_prev = analog_deg;
        return;
      }

      dt = (time_now - time_prev) / 1000.0;

      P = error;
      I += error * dt;
      D = -(analog_deg - analog_deg_prev) / dt;

      analog_deg_prev = analog_deg;
      time_prev = time_now;

      if (error < 30){
        correction = K_p * P + K_i * I + K_d * D;
      }
      else{
        correction = K_p * P + K_d * D;
      }
      
      //correction = constrain(correction, -20, 20);

      double servo_output = analog_deg + correction;
      //Serial.println(error);
      output_check = servo_output;

      servo_safe.check_and_set(servo_output);
    }

    // void modify_Kp(double value) { K_p = value; }
    // void modify_Ki(double value) { K_i = value; }
    // void modify_Kd(double value) { K_d = value; }
    float return_Kp(){
      return K_p;
    }
    
    float return_output(){
      return output_check;
    }

    float return_analog_deg(){
      return analog_deg;
    }

  private:
    double output_check;
    Servo_Safe_SET servo_safe;

    const double m, c;
    const double K_p;
    const double K_i;
    const double K_d;

    unsigned long time_prev = 0;
    unsigned long time_now;
    double dt;

    double analog_deg_prev = 0;
    double analog_deg;

    

    double P, I, D;
    double error;
    double correction;
};

class Analog_deg {
  public:
    Analog_deg(float servo_calibration_m,
              float servo_calibration_c)
      : m(servo_calibration_m),
        c(servo_calibration_c){}

    float return_deg(float analog_raw){
      analog_deg = (analog_raw - c) / m;
      return analog_deg;
    }

  private:
    const double m, c;
    float analog_deg;
};

//Analog_deg servo_joint0();
Analog_deg servo_joint1_deg(330.0 / 90.0, 502.0);
Analog_deg servo_joint2_deg(140.0/90.0,360);//197.0 / 90.0, 60.0);

Servo servo_hw0;
Servo servo_hw1;
Servo servo_hw2;

Servo servo_joint0;
Servo servo_joint1;
Servo servo_joint2;
Servo servo_pitch;
Servo servo_roll;
Servo servo_gripper;
//Servo_PID servo0(servo_hw0, "Base Servo", 3, 0, 360, 90, 192.0 / 90.0, 132.0, 0.9, 0.2, 0.0001); 
//Servo_PID servo1(servo_hw1, "Link 1", 4, 0, 360, 90, 330.0 / 90.0, 502.0, 1.6, 0.6, 0.0001); 
//Servo_PID servo2(servo_hw2, "Link 2", 5, 0, 360, 90, 197.0 / 90.0, 60.0, 1.0, 0.6, 0.00005); 
// info for servo 0: 192/90, 132, Kp 1, Ki 2, Kd 0.0001
// info for servo 1: 330/90, 502, Kp 1.6, Ki 2, Kd 0.0001
// info for servo 1: 197/90, 60, Kp 1.0, Ki 0.4, Kd 0.00005

const int analog_IN_servo_0 = A0;
const int analog_IN_servo_1 = A1;
const int analog_IN_servo_2 = A2;

float baseVal = 0, shoulderVal = 0, elbowVal = 0;
int basePin = A0, shoulderPin = A1, elbowPin = A8;
float L1 = 31.0, L2 = 17.0;

const int analog_IN_potentiometer = A8;

//int flag_nano = 0; // 1 = recieving, 0 = not
float Kx = 1;
float Ky = 1;
float Kz = 0.7;
float Ktheta = 1.4;
float Kphi = 1.2;
float X = 0, Y = 0, Z = 0, D = 0, theta = 0, phi = 0;
double desired_angle_servo0 = 90, desired_angle_servo1 = 90, desired_angle_servo2 = 90, desired_angle_servo3 = 45, desired_angle_servo4 = 90, desired_gripper_servo = 45;
float prev0=90, prev1=90, prev2=90, prev_theta = 90, prev_phi = 90, prev_grip = 90;
float max_step = 4.0, max_step_j2 = 2.0, max_step_grip = 10.0, max_step_wrist = 5.0;   // degrees per loop
float time_ms_prev = 0;
bool system_running = false;
String incomingString = "";

int N = 10;
float sum0, sum1, sum2, filtered_ave0, filtered_ave1, filtered_ave2;

float X_f = 0, Y_f = 0, Z_f = 0, D_prev = 0;
float X_prev = 0, Y_prev = 0, Z_prev = 0, theta_prev = 90, phi_prev = 90;
float theta_f = 0, phi_f = 0;
const float alpha_theta = 0.5, alpha_phi = 0.5;
const float alpha_xy = 0.5, alpha_z = 0.5;   // smoothing factor (0.1–0.3 good)


void sum_all(){
  sum0 = 0; sum1 = 0; sum2 = 0;
  for (int i = 0; i < N; i++) {
    sum0 += analogRead(analog_IN_servo_0);
    sum1 += analogRead(analog_IN_servo_1);
    sum2 += analogRead(analog_IN_servo_2);
  }
  filtered_ave0 = sum0 / float(N);
  filtered_ave1 = sum1 / float(N);
  filtered_ave2 = sum2 / float(N);
  return;
}

void update_xyzd(String data){
      // Parse the comma-separated values "X,Y,Z\n"
      int comma1 = data.indexOf(',');
      int comma2 = data.indexOf(',', comma1 + 1);
      int comma3 = data.indexOf(',', comma2 + 1);
      int comma4 = data.indexOf(',', comma3 + 1);
      int comma5 = data.indexOf(',', comma4 + 1);
      int comma6 = data.indexOf(',', comma5 + 1);//data.lastIndexOf(',');
      
      float X_cam = data.substring(0, comma1).toFloat();
      X = X_cam * (47.0/35.0)*Kx;
      float Y_cam = data.substring(comma1 + 1, comma2).toFloat();
      Y = (Y_cam + 15.0) * (47.0/30.0)*Ky;
      float Z_cam = data.substring(comma2 + 1, comma3).toFloat();
      Z = (Z_cam+7.0)*Kz;
      
 
      X_f = alpha_xy * X + (1 - alpha_xy) * X_f;
      Y_f = alpha_xy * Y + (1 - alpha_xy) * Y_f;
      Z_f = alpha_z * Z + (1 - alpha_z) * Z_f;
      
      if (abs(X_prev - X_f) > 0.5){
        X = X_f;
        X_prev = X_f;
      }
      else X = X_prev;
      if (abs(Y_prev - Y_f) > 0.5){
        Y = Y_f;
        Y_prev = Y_f;
      }
      else Y = Y_prev;
      if (abs(Z_prev - Z_f) > 0.5){
        Z = Z_f;
        Z_prev = Z_f;
      }
      else Z = Z_prev;
      

      //display_mine.print_OLED_4_vals(X, Y, Z, sqrt(X*X + Y*Y + Z*Z));
      X = constrain(X, -47.0, 47.0);
      Y = constrain(Y, 0.0, 47.0);
      Z = constrain(Z, -47.0, 47.0);
      
      D = data.substring(comma3 + 1, comma4).toFloat();
      if (abs(D - D_prev) < 1) D = D_prev;
      D_prev = D;

      theta = data.substring(comma4 + 1, comma5).toFloat();
      phi = data.substring(comma5 + 1,comma6).toFloat();
      if ((theta == 0) && (phi == 0)){
        theta = 90; 
        phi = 90;
      }
      theta_f = alpha_theta * theta + (1 - alpha_theta) * theta_f;
      phi_f = alpha_phi * phi + (1 - alpha_phi) * phi_f;
      if (abs(theta_prev - theta_f) > 2){
        theta = theta_f;
        theta_prev = theta_f;

      }
      else theta = theta_prev;
      if (abs(phi_prev - phi_f) > 2){
        phi = phi_f;
        phi_prev = phi_f;
      }
      else phi = phi_prev;
      float time_ms = data.substring(comma6+1).toFloat();
      float delta_time = time_ms - time_ms_prev;
      time_ms_prev = time_ms;
      Serial.println(delta_time);  // ← ADD THIS LINE
      display_mine.print_MPU_0x0(delta_time);
      float r = sqrt(X*X + Y*Y + Z*Z);

      if (r > 45) {
        float scale = 45 / r;
        X *= scale;
        Y *= scale;
        Z *= scale;
      }
      
      update_desired_angles();
}

float atan2_mine(float x, float y){
  if (x == 0){
    return 90.0;
  }
  else if (x > 0){
    return (180.0/PI)*atan(y/x);
    //return atan(y/x);
  }
  else if (x < 0){
    return 180.0+(180.0/PI)*atan(y/x);
    //return (PI/2)+atan(y/x);
  }
}


void update_desired_angles(){
  float phi1 = atan2_mine(X, Y);
  desired_angle_servo0 = phi1*(13.0/18.0)+25.0; // mapping of phi1 onto this system
  // float phi2 = (180.0/PI)*acos((((X*X)+(Y*Y)+(Z*Z))+961.0-289.0)/(2*31*(sqrt((X*X)+(Y*Y)+(Z*Z)))))+(180.0/PI)*asin(Z/sqrt((X*X)+(Y*Y)+(Z*Z)));
  float arg = (((X*X)+(Y*Y)+(Z*Z))+961.0-289.0)/(2*31*(sqrt((X*X)+(Y*Y)+(Z*Z))));
  arg = constrain(arg, -1.0, 1.0);
  float phi2 = (180.0/PI)*acos(arg) + (180.0/PI)*asin(Z/sqrt((X*X)+(Y*Y)+(Z*Z)));
  desired_angle_servo1 = phi2*(60.0/90.0)+15.0;
  // float phi3 = (180.0/PI)*acos((1250.0-((X*X)+(Y*Y)+(Z*Z)))/(1054.0));
  float arg2 = (1250.0-((X*X)+(Y*Y)+(Z*Z)))/(1054.0);
  arg2 = constrain(arg2,-1.0,1.0);
  float phi3 = 180.0 - ((180.0/PI)*acos(arg2));
  desired_angle_servo2 = phi3*(65.0/90.0);
  float angle1 = servo_joint1_deg.return_deg(filtered_ave1);
  float angle2 = servo_joint2_deg.return_deg(filtered_ave2);
  float phi4 = (Ktheta*theta+90) + angle1 - (180 - phi3);
  desired_angle_servo3 = phi4*(13.0/18.0)+25.0;
  float phi5 = Kphi*phi;
  desired_angle_servo4 = phi5*(134.0/180.0)+23.0;
  float phi6 = ((15.0-D)*155.0/15.0)+10.0; 
  phi6 = constrain(phi6, 20.0, 145.0);
  desired_gripper_servo = phi6;
  //display_mine.print_OLED_3_vals(phi1, phi2, phi3);
  //display_mine.print_OLED_3_vals(theta, angle1, phi3);
  
  float pot_value = analogRead(analog_IN_potentiometer);
  max_step = int(map(pot_value,0,1024,1,20));
  

  desired_angle_servo0 = constrain(desired_angle_servo0, prev0-max_step, prev0+max_step);
  desired_angle_servo1 = constrain(desired_angle_servo1, prev1-max_step, prev1+max_step);
  desired_angle_servo2 = constrain(desired_angle_servo2, prev2-max_step_j2, prev2+max_step_j2);
  desired_angle_servo3 = constrain(desired_angle_servo3, prev_theta-max_step_wrist, prev_theta+max_step_wrist);
  desired_angle_servo4 = constrain(desired_angle_servo4, prev_phi-max_step_wrist, prev_phi+max_step_wrist);
  desired_gripper_servo = constrain(desired_gripper_servo, prev_grip-max_step_grip, prev_grip+max_step_grip);
  

  prev0 = desired_angle_servo0;
  prev1 = desired_angle_servo1;
  prev2 = desired_angle_servo2;
  prev_theta = desired_angle_servo3;
  prev_phi = desired_angle_servo4;
  prev_grip = desired_gripper_servo;
}

void setup() {
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    // Most I2C OLEDs use address 0x3C
    for(;;); // Don’t proceed, loop forever
  }
  Serial.begin(115200);   // MUST match Raspberry Pi
  //servo0.begin();
  //servo1.begin();
  //servo2.begin();

  display.clearDisplay();
  
  // Set text properties
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  // Print text
  display.println("System Ready");
  display.println("128x64 I2C test");
  display.display();  // Show on screen

  servo_joint0.attach(3);
  servo_joint0.write(90);

  servo_joint1.attach(4);
  servo_joint1.write(90);

  servo_joint2.attach(5);
  servo_joint2.write(65);

  servo_pitch.attach(6);
  servo_pitch.write(25);
  
  servo_roll.attach(7);
  servo_roll.write(90);

  servo_gripper.attach(8);
  servo_gripper.write(155);
}


void loop() {
  
  //servo_pitch.write(90);

  while (Serial.available()) {
    String data = Serial.readStringUntil('\n');  // Reads until newline character
    //display_mine.print_MPU_0x0(data.c_str());
    update_xyzd(data);
    baseVal = (desired_angle_servo0-25.0)/(13.0/18.0);
    shoulderVal = (((analogRead(shoulderPin)-514)/3.6222));
    elbowVal = 180 - (((analogRead(elbowPin)-521)/3.9333));
    X = cos(baseVal*PI/180)*(L1*cos(shoulderVal*PI/180)+L2*cos(((180-elbowVal)-shoulderVal)*PI/180));
    Y = sin(baseVal*PI/180)*(L1*cos(shoulderVal*PI/180)+L2*cos(((180-elbowVal)-shoulderVal)*PI/180));
    Z = L1*sin(shoulderVal*PI/180)-L2*sin(((180-elbowVal)-shoulderVal)*PI/180)+14.0;
    Serial.print(millis());
    Serial.print(",");
    Serial.print(X);
    Serial.print(",");
    Serial.print(Y);
    Serial.print(",");
    Serial.println(Z);
    display_mine.print_OLED_3_vals(X, Y , Z);
    
  }
  
  sum_all();
  float angle1 = servo_joint1_deg.return_deg(filtered_ave1);
  float angle2 = servo_joint2_deg.return_deg(filtered_ave2);
  //display_mine.print_OLED_3_vals(0, angle1, angle2);
  //servo0.update_servo_with_PID(filtered_ave0, desired_angle_servo0);
  // Serial.print("Base set: ");
  // Serial.print(servo0.return_output());
  // Serial.print(", actual: ");
  // Serial.print(servo0.return_analog_deg());
  //servo1.update_servo_with_PID(filtered_ave1, desired_angle_servo1);
  // Serial.print(" || Joint 1 set: ");
  // Serial.print(servo1.return_output());
  // Serial.print(", actual: ");
  // Serial.print(servo1.return_analog_deg());
  // servo2.update_servo_with_PID(filtered_ave2, desired_angle_servo2);
  // Serial.print(" || Joint 2 set: ");
  // Serial.print(servo2.return_output());
  // Serial.print(", actual: ");
  // Serial.println(servo2.return_analog_deg());
  //display_mine.print_MPU_0x0(servo2.return_analog_deg());
  servo_joint0.write(desired_angle_servo0);
  servo_joint1.write(desired_angle_servo1);
  servo_joint2.write(desired_angle_servo2);
  servo_pitch.write(desired_angle_servo3);
  servo_roll.write(desired_angle_servo4);
  servo_gripper.write(desired_gripper_servo);
  //Serial.println(servo0.return_analog_deg());
  //delay(50);
}