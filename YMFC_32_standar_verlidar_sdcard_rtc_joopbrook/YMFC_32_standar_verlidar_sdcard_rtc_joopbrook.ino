///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
TwoWire HWire(2, I2C_FAST_MODE);          //Initiate I2C port 2 at 400kHz.
#include "SparkFun_VL53L1X.h"
#include <TimeLib.h>
#include <SPI.h>
#include <SD.h>

//rtc 
#include <DFRobot_DS1307.h>                 //Include RTC DS1307
DFRobot_DS1307 DS1307(&HWire, 0x68);

//filter 
#include <SimpleKalmanFilter.h>
SimpleKalmanFilter heightKalmanFilter(100, 25, 0.003);

#define SHUTDOWN_PIN -1
#define INTERRUPT_PIN -1 

SFEVL53L1X heightSensor(HWire, SHUTDOWN_PIN, INTERRUPT_PIN);
int16_t height;
int16_t filteredheight;

// #define DEBUG

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// float pid_p_gain_roll = 0;                //Gain setting for the pitch and roll P-controller (default = 1.3).
// float pid_i_gain_roll = 0;               //Gain setting for the pitch and roll I-controller (default = 0.04).
// float pid_d_gain_roll = 0;                 //Gain setting for the pitch and roll D-controller (default = 18.0).
// int pid_max_roll = 400;                     //Maximum output of the PID-controller (+/-).

float pid_p_gain_roll = 1.5;                //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.005;               //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 27;                 //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                     //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-).

// float pid_p_gain_yaw = 0;                //Gain setting for the pitch P-controller (default = 4.0).
// float pid_i_gain_yaw = 0;               //Gain setting for the pitch I-controller (default = 0.02).
// float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller (default = 0.0).
// int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 4;                   //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.02;                //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0;                   //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                      //Maximum output of the PID-controller (+/-).

// int16_t manual_acc_pitch_cal_value = 362;
// int16_t manual_acc_roll_cal_value = 170;
//Manual accelerometer calibration values for IMU angles:
int16_t manual_acc_pitch_cal_value = 186;
int16_t manual_acc_roll_cal_value = 150;

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = false;    // Set to false or true;
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;

uint8_t gyro_address = 0x69;               //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer

uint8_t last_channel_1, last_channel_2, last_channel_3, last_channel_4;
uint8_t highByte, lowByte, flip32, start;
uint8_t error, error_counter, error_led;

int16_t esc_1, esc_2, esc_3, esc_4, esc_average;
int16_t throttle, cal_int;
int16_t temperature, count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;

int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
uint8_t channel_select_counter;
int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

uint32_t elapsed_time; 
uint32_t loop_timer, error_timer, serial_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
float battery_voltage;
float angle_conversion;

//SD CARD
const uint8_t CS_PIN = PA4;                //Chipselect in stm32
File DataObject;
char fileName[] = "12345678.txt";
uint16_t getTimeBuff[7] = {0};
String x;
int seqNumber;
char buffer[120];
char outputarr[128];
int battery_voltageint;
int angle_roll_accint;
int angle_pitch_accint;
int angle_rollint;
int angle_pitchint; 
uint8_t save_sd_loop;
const uint8_t sd_loop_counter = 1;

boolean file_open = false;
boolean save_sd = false;
boolean auto_level;                       //Auto level on (true) or off (false).
boolean lidar_mode = true;                //lidar mati/hidup. false = mati. true = hidup
File SdFile; 

void dateTime(uint16_t* date, uint16_t* time)
{
  *date = FAT_DATE(getTimeBuff[6], getTimeBuff[5], getTimeBuff[4]);
  *time = FAT_TIME(getTimeBuff[2], getTimeBuff[1], getTimeBuff[0]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(PB1, INPUT_ANALOG);                                    //This is needed for reading the analog value of port A4.
  //Port PB3 and PB4 are used as JTDO and JNTRST by default.
  //The following function connects PB3 and PB4 to the
  //alternate output function.
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                    //Connects PB3 and PB4 to output function.

  //On the Flip32 the LEDs are connected differently. A check is needed for controlling the LEDs.
  pinMode(PB3, INPUT);                                         //Set PB3 as input.
  pinMode(PB4, INPUT);                                         //Set PB4 as input.
  if (digitalRead(PB3) || digitalRead(PB3))flip32 = 1;         //Input PB3 and PB4 are high on the Flip32
  else flip32 = 0;

  pinMode(PB3, OUTPUT);                                         //Set PB3 as output.
  pinMode(PB4, OUTPUT);                                         //Set PB4 as output.

  green_led(LOW);                                               //Set output PB3 low.
  red_led(HIGH);                                                //Set output PB4 high.
    
#ifdef DEBUG
  Serial.begin(57600);                                        //Set the serial output to 57600 kbps. (for debugging only)
  delay(250);                                                 //Give the serial port some time to start to prevent data loss.
#endif

#ifdef DEBUGGING
  Serial.begin(57600);                                        //Set the serial output to 57600 kbps. (for debugging only)
  delay(250);                                                 //Give the serial port some time to start to prevent data loss.
#endif

  timer_setup();                                                //Setup the timers for the receiver inputs and ESC's output.
  delay(50);                                                    //Give the timers some time to start.
  
  HWire.begin();                                                //Start the I2C as master  
  HWire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  error = HWire.endTransmission();                              //End the transmission and register the exit status.
  while (error != 0) {                                          //Stay in this loop because the MPU-6050 did not responde.
    error = 2;                                                  //Set the error status to 2.
    error_signal();                                             //Show the error via the red LED.
    delay(4);
  }

  gyro_setup();                                                 //Initiallize the gyro and set the correct registers.

  if (!use_manual_calibration) {
    //Create a 5 second delay before calibration.
    for (count_var = 0; count_var < 1250; count_var++) {        //1250 loops of 4 microseconds = 5 seconds
      if (count_var % 125 == 0) {                               //Every 125 loops (500ms).
        digitalWrite(PB4, !digitalRead(PB4));                   //Change the led status.
      }
      delay(4);                                                 //Delay 4 microseconds
    }
    count_var = 0;                                              //Set start back to 0.
  }

  calibrate_gyro();                                             //Calibrate the gyro offset.

  //Wait until the receiver is active.
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990)  {
    error = 3;                                                  //Set the error status to 3.
    error_signal();                                             //Show the error via the red LED.
    delay(4);
  }
  error = 0;                                                    //Reset the error status to 0.

  //Wait until the throtle is set to the lower position.
  while (channel_3 < 990 || channel_3 > 1050)  {
    error = 4;                                                  //Set the error status to 4.
    error_signal();                                             //Show the error via the red LED.
    delay(4);
  }
  error = 0;                                                    //Reset the error status to 0.

  if(lidar_mode){
    //error untuk pembacaan lidar
    error= heightSensor.begin();
    while (error != 0)
    {
      error = 6;
      error_signal();
      delay(4);
    }
    heightSensor.setDistanceModeLong(); 
    heightSensor.setTimingBudgetInMs(40);
    heightSensor.setIntermeasurementPeriod(48);
  }

  //When everything is done, turn off the led.
  red_led(LOW);                                                 //Set output PB4 low.

  //Load the battery voltage to the battery_voltage variable.
  //The STM32 uses a 12 bit analog to digital converter.
  //analogRead => 0 = 0V ..... 4095 = 3.3V
  //The voltage divider (1k & 10k) is 1:11.
  //analogRead => 0 = 0V ..... 4095 = 36.3V
  //36.3 / 4095 = 112.81.
  battery_voltage = (float)analogRead(PB1) / 112.81;

  SD.begin(CS_PIN);
  DS1307.begin();
  DS1307.stop();
  DS1307.start();
  DS1307.getTime(getTimeBuff);
  setTime(
              getTimeBuff[2],
              getTimeBuff[1],
              getTimeBuff[0],
              getTimeBuff[4],
              getTimeBuff[5],
              getTimeBuff[6]
              );
  SdFile::dateTimeCallback(dateTime);
  loop_timer = micros();                                        //Set the timer for the first loop.
  green_led(HIGH);                                              //Turn on the green led.
  elapsed_time = 4000;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  error_signal();                                                                  //Show the errors via the red LED.
  gyro_signalen();                                                                 //Read the gyro and accelerometer data.

  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //This is the added IMU code from the videos:
  //https://youtu.be/4BoIE8YQwM8
  //https://youtu.be/j-kE0AMEWy4
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  if (elapsed_time <= 4050) {
    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.
	
	angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
	angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
  } else {
    angle_pitch += (float)gyro_pitch * 1.52672E-08 * elapsed_time;
    angle_roll += (float)gyro_roll * 1.52672E-08 * elapsed_time;
	
	angle_pitch -= angle_roll * sin((float)gyro_yaw * 2.66463E-10 * elapsed_time);   //If the IMU has yawed transfer the roll angle to the pitch angel.
	angle_roll += angle_pitch * sin((float)gyro_yaw * 2.66463E-10 * elapsed_time);   //If the IMU has yawed transfer the pitch angle to the roll angel.
  }

  ////Gyro angle calculations
  ////0.0000611 = 1 / (250Hz / 65.5)
  //angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  //angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  ////0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
  //angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  //angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

  if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
  }
  if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
  }

  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.
  // angle_roll = angle_roll * 0.996 + angle_roll_acc * 0.004;

  pitch_level_adjust = angle_pitch * 15;                                           //Calculate the pitch angle correction.
  roll_level_adjust = angle_roll * 15;                                             //Calculate the roll angle correction.
  
  //set transmitter channel_5 = SA bawah 1101 autolevel
  if (channel_5 < 1200){
    auto_level = true; 
  }
  //set transmitter channel_5 = SA netral 1500us
  else {
    auto_level = false;
  } 
 //transmitter channel_5 = SA ATAS = 1998us

//transmitter channel_6 = 1500us  
  if (channel_6 < 1600) {
    save_sd = true; 
    #ifdef DEBUGGING
      Serial.println("save_sd_enabled");
    #endif
  }
//transmitter channel_6 = 1997/1998/1999 us
  else 
  {
    save_sd = false;;
  }

  if (!auto_level) {                                                               //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                        //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                         //Set the roll angle correcion to zero.
  }

  //For starting the motors: throttle low and yaw left (step 1).
  if (channel_3 < 1050 && channel_4 < 1050)start = 1;
  //When yaw stick is back in the center position start the motors (step 2).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {
    start = 2;

    green_led(LOW);                                                                //Turn off the green led.

    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }

  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    start = 0;
    green_led(HIGH);                                                               //Turn on the green led.
  }

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_1 > 1508)pid_roll_setpoint = channel_1 - 1508;
  else if (channel_1 < 1492)pid_roll_setpoint = channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_2 > 1508)pid_pitch_setpoint = channel_2 - 1508;
  else if (channel_2 < 1492)pid_pitch_setpoint = channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if (channel_3 > 1050) { //Do not yaw when turning off the motors.
    if (channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
    else if (channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
  }

  calculate_pid();                                                                 //PID inputs are known. So we can calculate the pid output.

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //1410.1 = 112.81 / 0.08.
  battery_voltage = battery_voltage * 0.92 + ((float)analogRead(PB1) / 1410.1);

  //Turn on the led if battery voltage is to low. In this case under 10.0V
  if (battery_voltage < 10.0 && error == 0)error = 1;
  

  throttle = channel_3;                                                            //We need the throttle signal as a base signal.

  if (start == 2) {                                                                //The motors are started.
    if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

    if (esc_1 < 1100) esc_1 = 1100;                                                //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                                //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                                //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                                //Keep the motors running.

    if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
    if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
    if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
    if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.

    if (save_sd){
      if (!file_open)
      {
        if (lidar_mode) heightSensor.startRanging();
        DS1307.getTime(getTimeBuff);
        sprintf (fileName, "%2d%2d%2d%2d.txt",
            getTimeBuff[4], //date, range : 01-31
            getTimeBuff[2], //hour, range : 00-23
            getTimeBuff[1], //min, range : 00-59
            getTimeBuff[0]  //sec, range : 00-59
        );

        x = String(fileName);
        x.replace(' ','0');
        x.toCharArray(fileName, 13);
        #ifdef DEBUG
          Serial.print("Nama FILE : ");
          Serial.println(fileName);
        #endif        
        DataObject = SD.open(fileName, FILE_WRITE);   
        if (DataObject) {
          #ifdef DEBUG
            Serial.print(fileName);
            Serial.println(" created");
          #endif
            file_open = true;
            save_sd_loop = 0;            
            DataObject.println("Gyro Calibration Value");
            DataObject.println("======================");
            DataObject.println("gyro roll calibration value;gyro pitch calibration value;gyro yaw calibration value");
            DataObject.print(manual_gyro_roll_cal_value);
            DataObject.print(";");
            DataObject.print(manual_gyro_pitch_cal_value);
            DataObject.print(";");
            DataObject.println(manual_gyro_yaw_cal_value);
            DataObject.println("      PID Values      ");
            DataObject.println("[PID Pitch = PID Roll]");
            DataObject.println("======================");
            DataObject.println("kp roll;ki roll;kd roll;kp yaw;ki yaw;kd yaw");
            DataObject.print(pid_p_gain_roll);
            DataObject.print(";");
            DataObject.print(pid_i_gain_roll);
            DataObject.print(";");
            DataObject.print(pid_d_gain_roll);
            DataObject.print(";");
            DataObject.print(pid_p_gain_yaw);
            DataObject.print(";");
            DataObject.print(pid_i_gain_yaw);
            DataObject.print(";");
            DataObject.println(pid_d_gain_yaw);
            DataObject.println("Sequence;time (us);raw acc pitch;raw acc roll;raw acc yaw;raw gyro roll;raw gyro pitch;raw gyro yaw;pitch angle(degree)*10;roll angle(degree)*10;acc pitch angle(degree)*10;acc roll angle(degree)*10;height(mm);ESC 1(us);ESC 2(us);ESC 3(us);ESC 4(us);receiver ch 1(roll);receiver ch 2(pitch);receiver ch 3(throttle);receiver ch 4(yaw);battery voltage (V)*10");
        }
        
        else {
          #ifdef DEBUG
          Serial.print("filename not created");        
          #endif
        } 
      }
    }
  }  

  else { // if start != 2
    esc_1 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for esc-1.
    esc_2 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for esc-2.
    esc_3 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for esc-3.
    esc_4 = 1000;                                                                  //If start is not 2 keep a 1000us pulse for esc-4.
     
    if (file_open && save_sd) {
      file_open = false;
      if (lidar_mode) heightSensor.stopRanging();
      DataObject.close();
    }
  }

  TIMER4_BASE->CCR1 = esc_1;                                                       //Set the throttle receiver input pulse to the ESC 1 output pulse.
  TIMER4_BASE->CCR2 = esc_2;                                                       //Set the throttle receiver input pulse to the ESC 2 output pulse.
  TIMER4_BASE->CCR3 = esc_3;                                                       //Set the throttle receiver input pulse to the ESC 3 output pulse.
  TIMER4_BASE->CCR4 = esc_4;                                                       //Set the throttle receiver input pulse to the ESC 4 output pulse.
  TIMER4_BASE->CNT = 5000;                                                         //This will reset timer 4 and the ESC pulses are directly created.

  if (lidar_mode){
    if (heightSensor.checkForDataReady())
    {
      //elapsed_time = micros();
      height = heightSensor.getDistance(); //Get the result of the measurement from the sensor
      filteredheight = heightKalmanFilter.updateEstimate(height);
      heightSensor.clearInterrupt();
      //elapsed_time = micros()- elapsed_time;
    }
  }
 
  if (file_open && save_sd) {
    save_sd_loop = save_sd_loop + 1;
    #ifdef DEBUGGING
      Serial.print("SD loop counter =");      
      Serial.println(save_sd_loop);
    #endif
    
    if (save_sd_loop == sd_loop_counter)
    {
      #ifdef DEBUGGING
        Serial.println("save_sd_enabled");
      #endif
      angle_pitchint = (int) (angle_pitch*10.0);
      angle_rollint= (int) (angle_roll*10.0);
      angle_pitch_accint = (int) (angle_pitch_acc*10.0);
      angle_roll_accint = (int) (angle_roll_acc*10.0);
      battery_voltageint = (int) (battery_voltage*10.0);
      esc_average = (esc_1+esc_2+esc_3+esc_4)/4;
      
      //angle_pitchstr = String(angle_pitch, 1);
      //angle_rollstr = String(angle_roll, 1);
      //angle_pitch_accstr = String(angle_pitch_acc, 1);
      //angle_roll_accstr = String(angle_roll_acc, 1);
      //heightstr = String(height/10.0, 1);
      //battery_voltagestr = String(battery_voltage, 1); 
      // snprintf(buffer, sizeof(buffer), "%i;%u;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i\r\n", 
      //           seqNumber, loop_timer, acc_y, acc_x, acc_z, 
      //           gyro_roll, gyro_pitch, gyro_yaw, 
      //           angle_pitchint, angle_rollint, 
      //           angle_pitch_accint, angle_roll_accint,
      //           height,esc_1, esc_2, esc_3, esc_4,
      //           channel_1, channel_2, channel_3, channel_4,
      //           battery_voltageint);
      snprintf(buffer, sizeof(buffer), "%i;%i;%i;%u%\r\n", 
                height,channel_3,esc_average,loop_timer
                );
                
      seqNumber++;
      if (seqNumber == 100) seqNumber = 0;
      save_sd_loop = 0;

      #ifdef DEBUGGING1
      if (serial_timer + 4 < millis()) {
        serial_timer = millis();
        // //Serial.print(F("Angle Pitch = "));
        // Serial.print(F("==AETR=="));    
        // Serial.print(channel_1);
        // Serial.print(F(";"));
        // Serial.print(channel_2);
        // Serial.print(F(";"));
        // Serial.print(channel_3);
        // Serial.print(F(";"));
        // Serial.print(channel_4);
        // //Serial.print(F(";"));
        // Serial.print(F("==GYRO=="));    
        // Serial.print(gyro_pitch_input);
        // Serial.print(F(";"));
        // Serial.print(gyro_roll_input);
        // Serial.print(F(";"));
        // Serial.print(gyro_yaw_input);
        // //Serial.print(F(";"));
        // Serial.print(F("==ANGLE=="));    
        // Serial.print(angle_pitch);
        // Serial.print(F(";"));
        // Serial.print(angle_pitch_acc);
        // Serial.print(F(";"));    
        // //Serial.print(F("Angle Roll = "));
        // Serial.print(angle_roll);
        // Serial.print(F(";"));
        // Serial.print(angle_roll_acc);
        // Serial.print(F("==ESC=="));
        // Serial.print(esc_1);
        // Serial.print(F(";"));
        // Serial.print(esc_2);
        // Serial.print(F(";"));
        // Serial.print(esc_3);
        // Serial.print(F(";"));
        // Serial.print(esc_4);
        // Serial.print(F("==PID=="));    
        // Serial.print(pid_output_pitch);
        // Serial.print(F(";"));
        // Serial.print(pid_output_roll);
        // Serial.print(F(";"));
        // Serial.print(pid_output_yaw);
        // Serial.print(F(";"));
        
        // //Serial.print(F("Elapsed time = "));
        // Serial.println(elapsed_time);
        Serial.print(gyro_roll);
        Serial.print(";");
        Serial.print(gyro_pitch);
        Serial.print(";");
        Serial.print(gyro_yaw);
        Serial.print(";");
        Serial.println(fileName);
      }
      #endif  
      ////////////////////////////////////////////////////////////////////////////////////////////////////
      //Creating the pulses for the ESC's is explained in this video:
      //https://youtu.be/Nju9rvZOjVQ
      ////////////////////////////////////////////////////////////////////////////////////////////////////

      //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
      //Because of the angle calculation the loop time is getting very important. If the loop time is
      //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure
      //that the loop time is still 4000us and no longer! More information can be found on
      //the Q&A page:
      //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
      #ifdef debug_on
        if (millis() - debug_timer > 1000) {
          debug_timer = millis();
          Serial.println(buffer);
        }
      #endif
      //elapsed_time = micros();
      unsigned int avail = DataObject.availableForWrite();
      if (avail >= strlen(buffer)) 
        DataObject.write(buffer,strlen(buffer));  
      else {
        // buffer[avail-2] = '\r';
        // buffer[avail-1] = '\n';
        DataObject.write(buffer,avail);
        
        if (DataObject.availableForWrite())       
        {
          DataObject.write(&buffer[avail],strlen(buffer)-avail);
        }
      }
      
    } 
    //  else 
    // {
    // save_sd_loop = save_sd_loop + 1;
    // }
  }
  
  elapsed_time = micros() - loop_timer;
  if (elapsed_time > 4050) {
    error = 5;                                                                      //Turn on the LED if the loop time exceeds 4050us.
  } else { 
    error = 0;  
    while (elapsed_time < 4000) elapsed_time = micros() - loop_timer;               //We wait until 4000us are passed.          
  }
  loop_timer = micros();                                                           //Set the timer for the next loop.
}
