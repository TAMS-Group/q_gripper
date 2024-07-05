/** SCS "diana gripper" driver and demo (nano33_diana_gripper_v1).
 *  
 *  Assumes 115200 baud for the native USB serial interface to the host 
 *  (Serial) and one 1 MBaud port for the SCServo bus at Serial1.
 *  Note that we power the SCservo TTLinker mini driver from 3V3 instead
 *  of the usual 5V, to protect the Arduino from Overvoltage. Otherwise,
 *  add a voltage divider (e.g. 1700Ohm 3300Ohm to the RX pin) and 
 *  solder the USB-V5 pin jumpoer on the Nano 33 IOT.
 * 
 *  Four "HX711" load-cell amplifiers are bit-banged, with  pin 2 (HX711_CLK1) 
 *  and pin 3 (HX711_DATA1) for the first amp, and pins (4,5) (6,7) (8,9)
 *  for the other ADCs.
 *  
 *  Optionally, a Neopixel RGB 12-LED ring can be connected to pin 11, 
 *  controlled via the Adafruit library.
 *  
 *  We also connect two three IMUs, the first one for on the gripper palm 
 *  (STM LSM6DS3, Arduino Nano 33 IOT on-board, address 0x6A). 
 *  Both fingertips mount a GY-521 breakout board with the MPU-6050 chip. 
 *  All IMUs are connected to the first I2C bus using pins A4/A5.
 *  
 *  Note that the GY-521 includes a 3.3V regulator on its VCC pin,
 *  so supply voltage should be 5V, while signal voltage for the
 *  MPU-6050 is 3.3V (but claimed to be 5V tolerant). 
 *  The I2C address is selected by the AD0 pin on the GY-521, low = 0x68 high = 0x69.
 *  The GY-521 comes with either 10K or 2K2 pullups for SDA/SCL; please check
 *  and if necessary replace the pullups with suitable resistors to enable
 *  stable operation.
 *  
 *  GY-521 circuit diagram:
 *  https://playground.arduino.cc/uploads/Main/MPU6050-V1-SCH/index.jpg 
 *  GY-521 hookup guild and tutorial:
 *  https://www.instructables.com/MPU6050-Arduino-6-Axis-Accelerometer-Gyro-GY-521-B/
 *  I2Cdev + MPU6050 libraries:
 *  https://github.com/jrowberg/i2cdevlib/zipball/master
 *  Default I2C address is 0x68 (pull-down), drive AD0 pin high for address 0x69
 *   
 *  Feetech SCS servo addresses assignment: the two finger gripper uses only one
 *  motor, default servo address is <12>.
 * 
 *  The mounted servo is a type SCS-20 and uses the newer SCS protocol
 *  with velocity/acceleration ramps and (coarse) torque and current
 *  readings. The Hall-effect position sensor is continuous with
 *  4096 steps (12-bits) per full rotation.
 *   
 *  SCS-15: idle current 0,     version L=5, H=15    SCS protocol
 *  SCS-20: idle current not 0, version L=9, H=5     SMS protocol
 *  
 *  Approximate acc/vel calibration results:
 *  vel [rad/sec]     = speed [counts] / 666  (* 0.0016)
 *  acc [rad/sec/sec] = accel [counts] * 0.16
 *  
 *  One SCS write transfer takes 6 .. 11 (0xff 0xff ID Fun Dat ... crc) bytes,
 *  or 60..110 usec at 1 MBaud. The read transfers also need to wait for the
 *  servo response. We set a default timeout of 500 usec for all bus transfers;
 *  you may need to increase this when you start using the sync* commands 
 *  that transmit more data.
 *  
 *  TODO:
 *  
 *  History:
 *  
 *  2022.07.09 - several cleanups, add "echo" mode
 *  2022.07.04 - copied from qbsc_gripper
 *  ...
 *  2021.11.26 - created (copy from ugly_hand_v7).
 *  
 *  (c) 2021, 2022 fnh, hendrich@informatik.uni-hamburg.de
 */
 
#include <math.h>
#include <stdlib.h>

#include <algorithm>
#include <vector>
#include <array>

#define  SCS_TIMEOUT_MICROS  500
#include <AsyncDelay.h>




/*
 * serial (bit-banged) interface to HX711 differential strain-gage ADC
 */
#define have_hx711   1
#define have_lf_imu  0   
#define N_LOADCELLS  (1)
#include "HX711_LITE.h"

HX711_LITE  loadcells[ N_LOADCELLS ];
long        loadcell_data[ N_LOADCELLS ] = { 0L };
long        loadcell_last_read_millis[ N_LOADCELLS ]  = { 0L };
int         loadcell_enabled[ N_LOADCELLS ]    = { 1 };
int         loadcell_errors[ N_LOADCELLS ]     = { 0 };
int         loadcell_clk_pins[ N_LOADCELLS ]  = { 2 };
int         loadcell_data_pins[ N_LOADCELLS ]  = { 3 };




/*
 * IMUs: one built-in LSM6DS3 IMU on board, two external IMUs
 * on the finger(tips).
 */

#include <Arduino_LSM6DS3.h>   // Note: this stupid library *hardcodes* an object called IMU

bool    have_palm_imu = true; // palm IMU is a LSM6DS3, on-board Nano 33 IOT
int     palm_imu_ax, palm_imu_ay, palm_imu_az;
int     palm_imu_gx, palm_imu_gy, palm_imu_gz;


// two InvenSense 6050 6-axis IMUs at I2C addresses 0x68 and 0x69. 
// 
//#include <I2Cdev.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// if needed, scale IMU values back from float to (signed int 16),
// current library sets 2000 deg/sec gyro range and 4G acc range, 104 Hz,
// see ~/Arduino/Libaries/Arduino_Arduino_LSM6DS3/src for sources...
// 
#define GYRO_TO_INT   (32768.0 / 2000.0)
#define ACC_TO_INT    (32768.0 / 4.0)


/*  
 * Feetech SCServo stuff 
  */
#include "SCSCL.h"
#include "SMSCL.h"

SCSCL scs; // older/smaller SC servos
SMSCL sms; // newer/bigger SM servos

#define LED_PIN  (13)

#define SCS_UNKNOWN 0
#define SCS_SERVO 1
#define SMS_SERVO 2
#define SMM_SERVO 3

// the q_gripper has one SCS-20 servo: Finger_Joint
// 
#define N_JOINTS  (1)
#define N_SERVOS  (1)

#define DEG2RAD  (PI/180.0)
#define RAD2DEG  (180.0/PI)
#define DEG2COUNTS  (4096.0/360.0)
#define COUNTS2DEG  (360.0/4096.0)



// abbreviations for all servo(s) (joints)
//
#define FJ           (0)


// factory calibration values (joint zero degrees -> Hall-effect counts)
// 
#define FJ      2048


typedef enum command_mode {
  idle_mode = 0,
  position_mode = 1,
  velocity_mode = 2,
  trajectory_mode = 3,
  effort_mode = 4,
  teach_mode = 5,
} command_mode_t;


command_mode_t  active_mode = idle_mode;


int  demo_enable = 0;
int  echo_mode = 0; 
int  led_val = 0;
int  t_blink = 0;
int  t_last_command = -3000;
int  t_last_TUV = 0;
int  t_last_temperatures = 0;
int  t_last_voltages = 0;
int  t_last_currents = 0;
int  t_last_torques = 0;
int  t_last_forces = 0;

int  t_last_palm_imu  = 0;
int  t_last_lf_imu    = 0;
int  t_last_rf_imu    = 0;

int  sid = 3;


// note: typical servos specified with 60 degrees ~ 0.1 sec, 
// or 600 degrees / second (max).
// For SMS with Hall-effect sensor, 360 deg = 4096 position counts,
// (angular resolution ~ 0.1 degrees), or about 6800 counts/sec.
// 
int wpspeed = 500; // 10..100 = rather slow; 1000 mid, 5000 fast
int wpaccel = 500; // 10 noticeably soft, 100 ok, 1000 fast
int wptime = 1000;

int verbose = 2;
bool enableDebugMsgs = false;


String serialCommandBuffer = "";


int t_now = 0;
int iteration = 0;
int seq_num = 0;

#define N_SERVOS 1

String servo_names[ N_SERVOS ]          = { "FJ" };
int    servo_ids[ N_SERVOS ]            = {     37}; // 36 q_gripper_l, 37 q_gripper_r
int    servo_types[N_SERVOS]            = { SMS_SERVO};
int    servo_open_positions[ N_SERVOS ] = {   2250};
int    servo_zero_positions[ N_SERVOS ] = {   1900}; // 1970 q_gripper_l, 1900 q_gripper_r
int    servo_flex_positions[ N_SERVOS ] = {   1380};
int    servo_signs[ N_SERVOS ]          = {     -1}; // +1: increase on close
int    servo_resolution[ N_SERVOS ]     = {   4096}; // counts per 2pi

int    servo_enabled[ N_SERVOS ]        = {      1};
int    servo_timeouts[ N_SERVOS ]       = {      0};
int    servo_torque_limits[ N_SERVOS ]  = {    512}; // not yet used

       // defaults are medium acceleration and fast speed (range 1..4095 each)
int    servo_accels[ N_SERVOS ]         = {    100};
int    servo_speeds[ N_SERVOS ]         = {    500};

int    joint_position_counts[ N_SERVOS ];
int    joint_velocity_counts[ N_SERVOS ];
int    joint_torque_counts[ N_SERVOS ];
int    last_joint_position_update = 0;

int    joint_position_goals[ N_SERVOS ];
int    joint_position_errors[ N_SERVOS ];



double joint_radians[N_SERVOS];             // radians
double joint_velocities[ N_SERVOS ];        // rad/sec
double joint_torques[ N_SERVOS ];           // Nm

bool   joint_position_counts_initialized = false;
bool   joint_position_goals_initialized = false;



AsyncDelay  scsWatchdog;
AsyncDelay  demoDelay; // start(3000, AsyncDelay::MILLIS); isExpired(); repeat() (from expired); restart() from now

char   print_buffer[128];

void usage() 
{
  Serial.println( "h / ?     print this help" );
  Serial.println( "f         print load cell forces" );
  Serial.println( "p         print servo position (SCS)" );
  Serial.println( "q         print servo position (SMS)" );
  Serial.println();
  Serial.println( "t         print servo temperature" );
  Serial.println( "i         print motor current (SCS)" );
  Serial.println( "j         print motor current (SMS)" );
  Serial.println( "r         <id> <addr> read register value" );
  Serial.println( "v         print supply voltage" );
  Serial.println( "w         <id> <addr> <value>  write register value" );
  Serial.println( "$         print temp + voltages + currents" );
  Serial.println( "#         <int>*7 enable/disable servos" );
  Serial.println( "@         0/1 enable echo mode (test without servos)" );
  Serial.println();
  Serial.println( "A         <int>*1 set servo accel(s) (SMS) [0 .. 4095]" );
  Serial.println( "B         <int>*1 set servo speed(s) (SMS) [0 .. 4095]" );
  Serial.println( "C         <int> close fingers by given degrees" );
  Serial.println( "D         start/stop/continue demo" );
  Serial.println( "E         <int>*1 effort goal (counts)" );
  Serial.println( "F         freeze (at current position)" );
  Serial.println( "G         <int>*1 position goal (counts)" );
  Serial.println( "H         move to home (all zero)" );
  Serial.println();
  Serial.println( "M         set mode: 0=idle 1=pos 2=vel 3=teach" );
  Serial.println( "O         <int> open fingers by given degrees" );
  Serial.println( "P         <int> parallel-grasp pos [1500..2500]" );
  
  Serial.println( "Q         <pos> set servo goal position (SMS) [0 .. 4095]" );
  Serial.println( "Q         <pos> <vel> <acc> set servo goal position (SMS) [0 .. 4095]" );
  // Serial.println( "R         set publishing rate (1/sec) [1 .. 100]" );
  Serial.println( "S         select active servo id (3..253)" );
  Serial.println( "T         <int>*7  set torque limit (counts)" );
  // Serial.println( "U         to be determined" );
  Serial.println( "Y         <int> set verbose level" );
  Serial.println( "Z         set zero bias (load cells)" );
  
  Serial.println( "d / ESC   disable demo (+ freeze) " );
}


void demoDebugMsg( String s ) {
  if (enableDebugMsgs) Serial.print( s ); 
}


void demoDebugMsg2( String s, int value ) {
  if (enableDebugMsgs) { Serial.print( s ); Serial.println( value ); }
}


/**
 * called in reaction to a broken/non-communicating servo.
 * Increments the servo_timeouts[] count for the given servo,
 * and disables the servo when a count of 5 is reached.
 */
void handleServoTimeout( int j ) {
  servo_timeouts[j] ++;
  if (verbose > 1) {
    Serial.print( "### servo timeout, index " );
    Serial.print( j ); 
    Serial.print( " servo-ID " );
    Serial.print( servo_ids[j] );
    Serial.print( " n_timeouts i" );
    Serial.println( servo_timeouts[j] );
  }
  
  if (servo_timeouts[j] > 5) {
    servo_enabled[j] = 0;
    Serial.print( "### servo disabled, index " );
    Serial.print( j ); 
    Serial.print( " servo-ID " );
    Serial.print( servo_ids[j] );
    Serial.print( " n_timeouts i" );
    Serial.println( servo_timeouts[j] );
  }
}


/** 
 * convert the current joint_angle from Feetech "counts" to radians.
 */
float get_joint_angle( int id ) {
  int pos = joint_position_counts[id];

  return 1.0 * pos / servo_resolution[id];
}




/**
 * send a new position goal (in Feetech servo counts) for all servos.
 */
void setJointPositionGoal( int goal_positions[N_SERVOS] )
{
  for( int j=0; j < N_SERVOS; j++ ) {
    joint_position_goals[j] = goal_positions[j];

    if (servo_enabled[j] != 1) continue; // skip disabled servos

    if (servo_types[j] == SCS_SERVO) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      scs.WritePos( servo_ids[j], goal_positions[j], 100, servo_speeds[j] );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    else if (servo_types[j] == SMS_SERVO) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      sms.WritePosEx( servo_ids[j], goal_positions[j], 
                      servo_speeds[j], servo_accels[j] ); // id, position, speed, accel
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    else {
      Serial.print( "... unknown servo type " );
      Serial.print( servo_types[j] );
      Serial.println( ", ignored." );
    }
  }
  joint_position_goals_initialized = true;

  if (verbose > 3) {
    Serial.print( "... jointPositionGoal[0]: " );
    Serial.println( goal_positions[0] );
    Serial.flush();              
  }
}


void setServoTorqueLimits( int torque_limits[N_SERVOS] )
{
  if (verbose > 4) Serial.println( "setServoTorqueLimits..." );
  for( int j=0; j < N_SERVOS; j++ ) {
    servo_torque_limits[j] = torque_limits[j];
  }
}



void printStampedIntArray( char* prefix, char* mnemonic, int* data ) 
{
  sprintf( print_buffer, "%s %d %d %s %d \n",
                 prefix, 
                 (seq_num++) & 0xff, 
                 millis() & 0xff, 
                 mnemonic, 
                 data[0]);
  Serial.print( print_buffer );
  Serial.flush();                 
}


void printStampedLongArray( char* prefix, char* mnemonic, long* data ) 
{
  sprintf( print_buffer, "%s %d %d %s %ld \n",
                 prefix, 
                 (seq_num++) & 0xff, 
                 millis() & 0xff, 
                 mnemonic, 
                 data[0]);
  Serial.print( print_buffer );
  Serial.flush();                 
}




/**
 * (re-) initialize the SCS bus and try to autodetect all connected servos.
 */
void initSCS() {
  Serial.print( "... initSCS..." );

  Serial1.begin(1000*1000);
  scs.pSerial = &Serial1;
  sms.pSerial = &Serial1;

  int reply;
  int misses = 0;
  for( int id=2; id <= 25; id++ ) { // 253!
    reply = scs.Ping( id );
    if (reply == id) {
      sprintf( print_buffer, "... Servo found at ID %3d %2x ping %d\n", id, id, reply );
      Serial.print( print_buffer ); Serial.flush();

      dumpServoData( id );
      misses = 0;
    }
    else {
      Serial.print( "." );
      misses ++; 
      if (misses > 32) { Serial.println(); misses = 0; }
    }
  }
}



/**
 * reads the current servo positions (in counts) and prints 
 * them in human-readable format.
 */
void sendServoPositions() {
  if (verbose > 4) Serial.print( "... sendServoPositions...\n" );

  int t0 = micros();

  if (echo_mode) {
    printStampedIntArray( "P", "pos", joint_position_goals );
    delay( 8 );
    return;
  }

  for( int j=0; j < N_SERVOS; j++ ) {
    int id   = servo_ids[ j ];
    int type = servo_types[ j ];
    
    // if (verbose > 6) Serial.printf( "... j id type   %d  %d  %d\n", j, id, type );

    if (servo_enabled[j] != 1) {
       // skip, keep latest position estimate intact
       continue;
    }

    if (type == SCS_SERVO) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      joint_position_counts[j] = scs.ReadPos( id );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );

    }
    else if (type == SMS_SERVO) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      joint_position_counts[j] = sms.ReadPos( id );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    else {
      Serial.print( "... unknown servo type " );
      Serial.println( type );
      Serial.flush();              
    }
  }
  int t2 = micros();

  // Serial.printf( "P %d %d pos %d \n",
  printStampedIntArray( "P", "pos", joint_position_counts );

  joint_position_counts_initialized = true;
  
  if (verbose > 4) {
    sprintf( print_buffer, "... took %d us.\n", (t2-t0) );
    Serial.print( print_buffer );
    Serial.flush();
  }

}


void sendServoVelocities() {
  if (verbose > 4) Serial.print( "... sendServoVelocities...\n" );

  int t0 = micros();

  for( int j=0; j < N_SERVOS; j++ ) {
    int id   = servo_ids[ j ];
    int type = servo_types[ j ];
    
    if (verbose > 6) {
      sprintf( print_buffer, "... j id type   %d  %d  %d\n", j, id, type );
      Serial.print( print_buffer ); Serial.flush();
    }

    if (servo_enabled[j] != 1) {
       // skip disabled servo, keep latest velocity_counts
       continue;
    }

    if (type == SCS_SERVO) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      joint_velocity_counts[j] = scs.ReadSpeed( id );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    else if (type == SMS_SERVO) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      joint_velocity_counts[j] = sms.ReadSpeed( id );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    else {
      sprintf( print_buffer, "... Unknown servo type: %d ignored.\n", type );
      Serial.print( print_buffer ); Serial.flush();
    }
    
    joint_radians[j] = (joint_position_counts[j] - servo_zero_positions[j]) 
                       * servo_signs[j] * COUNTS2DEG;
  }
  int t2 = micros();

  // sprintf( print_buffer, "V %d %d vel %d \n",
  printStampedIntArray( "V", "vel", joint_velocity_counts );
  
  if (verbose > 4) {
    sprintf( print_buffer, "... took %d us.\n", (t2-t0) );
    Serial.print( print_buffer ); Serial.flush();
  }
}


void sendServoTorques() {
  if (verbose > 4) Serial.print( "... sendServoTorques...\n" );

  int t0 = micros();

  for( int j=0; j < N_SERVOS; j++ ) {
    int id   = servo_ids[ j ];
    int type = servo_types[ j ];
    if (verbose > 6) {
      sprintf( print_buffer, "... j id type   %d  %d  %d\n", j, id, type );
      Serial.print( print_buffer ); Serial.flush();
    }

    if (servo_enabled[j] != 1) {
       // skip disabled servo, keep latest torque readings
       continue;
    }

    if (type == SCS_SERVO) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      joint_torque_counts[j]   = scs.ReadLoad( id );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    else if (type == SMS_SERVO) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      joint_torque_counts[j]   = sms.ReadLoad( id );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
   }
    else {
      sprintf( print_buffer, "... Unknown servo type: %d ignored.\n", type );
      Serial.print( print_buffer ); Serial.flush();
    }
  }

  int t2 = micros();

  // sprintf( print_buffer, "M %d %d trq %d \n",
  printStampedIntArray( "M", "trq", joint_torque_counts );
  
  if (verbose > 4) {
    sprintf( print_buffer, "... took %d us.\n", (t2-t0) );
    Serial.print( print_buffer ); Serial.flush();
  }
}


void sendServoPositionErrors() {
  if (verbose > 4) Serial.print( "... sendServoPositionErrors...\n" );

  int t0 = micros();

  if (!joint_position_counts_initialized) return;

  if (!joint_position_goals_initialized) {
    for( int j=0; j < N_SERVOS; j++ ) {
       joint_position_goals[j] = joint_position_counts[j];
    }  
    joint_position_goals_initialized = true;
  }

  for( int j=0; j < N_SERVOS; j++ ) {
    joint_position_errors[j] = joint_position_counts[j] - joint_position_goals[j];
   
    if (verbose > 6) {
      sprintf( print_buffer, "... j os goal err  %d  %d  %d  %d\n", 
                                    j, joint_position_counts[j], joint_position_goals[j],
                                       joint_position_errors[j] );
      Serial.print( print_buffer ); Serial.flush();
    }
  }

  int t2 = micros();

  {
    // sprintf( print_buffer, "E %d %d pos %d \n",
    printStampedIntArray( "E", "pos", joint_position_errors );
  }
  
  if (verbose > 4) {
    sprintf( print_buffer, "... took %d us.\n", (t2-t0) );
    Serial.print( print_buffer ); Serial.flush();
  }
}




void sendServoTemperatures() {
  if (verbose > 4) Serial.print( "... sendServoTemperatures...\n" );

  int t0 = micros();
  if ((t0 - t_last_temperatures) < 50000) return;
  t_last_temperatures = t0;

  int temp[N_SERVOS];

  for( int j = 0; j < N_SERVOS; j++ ) {
    if (servo_enabled[j] == 1) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      temp[j] = scs.ReadTemper( servo_ids[j] );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    // else keep latest temperature reading
  }
  int t1 = micros();

  if (verbose > 4) {
    sprintf( print_buffer, "... sendServoTemperatures took %d usec.\n", (t1-t0) );                   
    Serial.print( print_buffer ); Serial.flush();
  }

  // sprintf( print_buffer, "T %d %d temp %d \n",
  printStampedIntArray( "T", "temp", temp );
}


void sendServoVoltages() {
  if (verbose > 4) Serial.print( "... sendServoVoltages...\n" );

  int t0 = micros(); 

  if ((t0 - t_last_voltages) < 50000) return;
  t_last_voltages = t0;

  int volts[N_SERVOS];

  for( int j=0; j < N_SERVOS; j++ ) {
    if (servo_enabled[j] == 1) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      volts[j] = scs.ReadVoltage( servo_ids[j] );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    // else keep latest voltage
  }
  
  int t1 = micros();

  if (verbose > 4) {
   sprintf( print_buffer, "... sendServoVoltages took %d usec.\n", (t1-t0) );                   
   Serial.print( print_buffer ); Serial.flush();
  }

  // sprintf( print_buffer, "U %d %d volts %d \n",
  printStampedIntArray( "U", "volts", volts );
}


void sendServoCurrents() {
  if (verbose > 4) Serial.print( "... sendServoCurrents...\n" );

  int t0 = micros();
  if ((t0 - t_last_currents) < 50000) return;
  t_last_currents = t0;

  unsigned int currents[N_SERVOS]; // note: using int sometimes results in negative values

  for( int j=0; j < N_SERVOS; j++ ) {
    if (servo_enabled[j] == 1) {
      scsWatchdog.start( SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      currents[j] = scs.ReadCurrent( servo_ids[j] );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    // else keep latest current
  }

  int t1 = micros();

  if (verbose > 4) {
    sprintf( print_buffer, "... sendServoCurrents took %d usec.\n", (t1-t0) );                   
    Serial.print( print_buffer ); Serial.flush();
  }

  // sprintf( print_buffer,  "I %d %d amps %d \n",
  printStampedIntArray( "I", "amps", (int*) currents );
}


/**
 * send measured forces to the host. This function is called every loop cycle,
 * but actual sensor sampling is auto-throttled to the ADC capabilities, that is
 * 80 Hz for the HX711 ones, and full rate for ADS126X sensors.
 */
void sendStrainGageForces() {
  if (verbose > 4) Serial.print( "... sendStrainGageForces...\n" );

  // HX711 sensors, sample rate is about 80 Hz max
  //
  long t = millis(); long limit = (1 << 20);

  if (have_hx711) {

    for( int i=0; i < N_LOADCELLS; i++ ) {
      if (!loadcell_enabled[i]) {
        if (verbose > 5) {
          sprintf( print_buffer, "... loadcell %d disabled, skipping...\n", i );
          Serial.println( print_buffer ); Serial.flush();
        }
        continue;
      }

      if (loadcells[i].is_ready()) {
        long raw = loadcells[i].read();
        if (verbose > 3) {
          sprintf( print_buffer, "... loadcell %d got raw data %ld\n", i, raw );
          Serial.println( print_buffer ); Serial.flush();
        }

        if ((raw > limit) || (raw < -limit)) {
          sprintf( print_buffer, "### HX711 %d glitch, ignored: got raw %ld", i, raw );
          Serial.println( print_buffer ); Serial.flush();
        }
        else {
          loadcell_data[i] = raw;
          loadcell_last_read_millis[i] = millis();
        }
      } // is_ready
    } // for i

  }
  else { // fake data to test without hardware connected

    for( int i=0; i < N_LOADCELLS; i++ ) {
      loadcell_data[i] = (i*10000) + (t % (i*3000));
      loadcell_last_read_millis[i] = millis();
    }
  }

  // sprintf( print_buffer, "F %d %d forces %ld %ld \n", 
  printStampedLongArray( "F", "forces", loadcell_data );
}



void updateJointAngles() {
  if (verbose > 4) Serial.print( "... updateJointAngles...\n" );

  int t0 = micros();

  for( int j=0; j < N_SERVOS; j++ ) {
    int id   = servo_ids[ j ];
    int type = servo_types[ j ];
    if (verbose > 6) {
      sprintf( print_buffer, "... j id type   %d  %d  %d\n", j, id, type );
      Serial.print( print_buffer ); Serial.flush();
    }

    if (servo_enabled[j] != 1) continue;

    if (type == SCS_SERVO) {
      scsWatchdog.start( 2*SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      joint_position_counts[j] = scs.ReadPos( id );
      joint_velocity_counts[j] = scs.ReadSpeed( id );
      joint_torque_counts[j]   = scs.ReadLoad( id );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
    }
    else if (type == SMS_SERVO) {
      scsWatchdog.start( 2*SCS_TIMEOUT_MICROS, AsyncDelay::MICROS);
      joint_position_counts[j] = sms.ReadPos( id );
      joint_velocity_counts[j] = sms.ReadSpeed( id );
      joint_torque_counts[j]   = sms.ReadLoad( id );
      if (scsWatchdog.isExpired()) handleServoTimeout( j );
   }
    else {
      sprintf( print_buffer, "... Unknown servo type: %d ignored.\n", type );
      Serial.print( print_buffer ); Serial.flush();
    }
    
    joint_radians[j] = (joint_position_counts[j] - servo_zero_positions[j]) 
                       * servo_signs[j] * COUNTS2DEG;
  }
  int t2 = micros();

  if (verbose > 4) {
    for( int j=0; j < N_SERVOS; j++ ) {
      sprintf( print_buffer, "... servo %2d type %1d pos %5d  speed %5d  torque %5d  deg %7.3f \n", 
                   servo_ids[j], servo_types[j],
                   joint_position_counts[j],
                   joint_velocity_counts[j],
                   joint_torque_counts[j],
                   joint_radians[j] );
      Serial.print( print_buffer ); Serial.flush();
    }
  }

  sprintf( print_buffer, "D %d %d J %d V %d E %d \n",
                 iteration & 0xff, 
                 millis() & 0xff, 
                 joint_position_counts[0],
                 joint_velocity_counts[0],
                 joint_torque_counts[0] );
  Serial.print( print_buffer ); Serial.flush();
  
  if (verbose > 4) {
    sprintf( print_buffer, "... took %d us.\n", (t2-t0) );
    Serial.print( print_buffer ); Serial.flush();
  }
}




void dumpServoData( int sid ) {
  int baudrate    = scs.readByte( sid, SCSCL_BAUD_RATE );
  int version_l   = scs.readByte( sid, SMSCL_VERSION_L );
  int version_h   = scs.readByte( sid, SMSCL_VERSION_H );
  int current     = scs.ReadCurrent( sid );

  sprintf( print_buffer, "... baud rate (index): %d version L/H %d %d\n", baudrate, version_l, version_h );
  Serial.print( print_buffer ); Serial.flush();

  int pos         = scs.ReadPos( sid );
  int speed       = scs.ReadSpeed( sid );
  int torque      = scs.ReadLoad( sid );
  int voltage     = scs.ReadVoltage( sid );
  // int current     = scs.ReadCurrent( sid );
  int temperature = scs.ReadTemper( sid );
  sprintf( print_buffer, "... pos %d speed %d load %d voltage %d current %d temperature %d\n",
                 pos, speed, torque, voltage, current, temperature );
  Serial.print( print_buffer ); Serial.flush();

  int min_angle_l = scs.readByte( sid, SCSCL_MIN_ANGLE_LIMIT_L );
  int min_angle_h = scs.readByte( sid, SCSCL_MIN_ANGLE_LIMIT_H );
  int max_angle_l = scs.readByte( sid, SCSCL_MAX_ANGLE_LIMIT_L );
  int max_angle_h = scs.readByte( sid, SCSCL_MAX_ANGLE_LIMIT_H );

  sprintf( print_buffer, "... min angle: %d %d  max angle: %d %d\n",
                 min_angle_h, min_angle_l, max_angle_h, max_angle_l );
  Serial.print( print_buffer ); Serial.flush();

  int cw_dead      = scs.readByte( sid, SCSCL_CW_DEAD );
  int ccw_dead     = scs.readByte( sid, SCSCL_CW_DEAD );
  int pid_p        = scs.readByte( sid, SMSCL_COMPLIANCE_P ); // 21
  int pid_i        = scs.readByte( sid, SMSCL_COMPLIANCE_I ); // 22
  int pid_d        = scs.readByte( sid, SMSCL_COMPLIANCE_D ); // 23

  sprintf( print_buffer, "... deadband: cw %d  ccw %d  pid %d %d %d \n", 
                 cw_dead, ccw_dead, pid_p, pid_i, pid_d );
  Serial.print( print_buffer ); Serial.flush();

  int goal_pos_l   = scs.readByte( sid, SCSCL_GOAL_POSITION_L );
  int goal_pos_h   = scs.readByte( sid, SCSCL_GOAL_POSITION_H );
  int goal_time_l  = scs.readByte( sid, SCSCL_GOAL_TIME_L );
  int goal_time_h  = scs.readByte( sid, SCSCL_GOAL_TIME_H );
  int goal_speed_l = scs.readByte( sid, SCSCL_GOAL_SPEED_L );
  int goal_speed_h = scs.readByte( sid, SCSCL_GOAL_SPEED_H );

  sprintf( print_buffer, "... goal pos: %d %d  time: %d %d  speed: %d %d\n",
                 goal_pos_h, goal_pos_l, goal_time_h, goal_time_l,
                 goal_speed_h, goal_speed_l );
  Serial.print( print_buffer ); Serial.flush();

  String type;

  if      ((current != 0) && (version_l == 9))  type = "SCS_20";
  else if ((current == 0) && (version_h == 15)) type = "SCS_15";
  else if ((current == 0) && (version_h == 4))  type = "SCS_09";
  else if ((current == 0) && (version_l == 9)) { type = "SCS_20"; 
  
  Serial.println( "SERVO 14 MISBEHAVES AGAIN..." ); 
  
  }
  else                                          type = "UNKNOWN";

  Serial.print( "... detected servo type is: " );
  Serial.println( type );
  Serial.println();
  Serial.flush();
}





void freeze_now() 
{
  Serial.println( "freeze_now(): freezing at current positions\n" );
  setJointPositionGoal( joint_position_counts );
  delay( 1000 );
}



int fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  float tmp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min + 0.5;
  return (int) tmp;
}


float clamp( float value, float min_value, float max_value ) {
  if (value >= max_value) return max_value;
  if (value <= min_value) return min_value;
  return value;
}


/* move the selected servo (finger) to the given joint angle (degrees)
 * within the given duration (milliseconds)
 */
void move_to_degrees( int id, float angle, int duration ) {
  sprintf( print_buffer, "move_to: finger %d angle %8.3f duration %d\n",
                id, angle, duration );
  Serial.print( print_buffer ); Serial.flush();

  float offset = (servo_flex_positions[ id ] - servo_zero_positions[ id ])
               * (angle - 0.0);

  int goal = servo_zero_positions[ id ] + (int) ( offset );
  sprintf( print_buffer, "... goal= %d\n", goal );
  Serial.print( print_buffer ); Serial.flush();

  joint_position_goals[id] = goal;
 
  switch( servo_types[id] ) {

    case SMS_SERVO:
                sms.WritePosEx( servo_ids[id], goal, servo_speeds[ id ], servo_accels[ id ] );
                break;

    case SCS_SERVO:      
                scs.WritePos( servo_ids[id], goal, 100, servo_speeds[id] );
                break;
                
    default:
                sprintf( print_buffer, "Invalid finger_id %d in move_to_degrees, ignored\n", id );
                Serial.print( print_buffer ); Serial.flush();
  }
}


void straight_fingers() {
  demoDebugMsg( "straight fingers..." );
  for( int i=0; i < N_SERVOS; i++ ) {
    move_to_degrees( i, 0.0, 1000 ); // index, angle, speed/time
  }
}


void open_fingers() {
  demoDebugMsg( "open_fingers (2 degrees)..." );
  for( int i=0; i < N_SERVOS; i++ ) {
    float angle = get_joint_angle( i );
    move_to_degrees( i, angle-2.0, 1000 ); // index, angle, speed/time
  }
}


void close_fingers() {
  demoDebugMsg( "close_fingers (2 degrees)..." );
  for( int i=0; i < N_SERVOS; i++ ) {
    float angle = get_joint_angle( i );
    move_to_degrees( i, angle+2.0, 1000 ); // index, angle, speed/time
  }
}


// map(value, fromLow, fromHigh, toLow, toHigh)



/**
 * send raw IMU values (accelerometer, gyroscope, maybe compass).
 * currently using two IMUs (middle-finger and thumb).
 */
void sendAccelGyro() {
  if (verbose > 4) Serial.print( "... sendAccelGyro...\n" );

  int t0 = micros();

  if (have_palm_imu) {
    float _gx, _gy, _gz;
    float _ax, _ay, _az;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope( _gx, _gy, _gz ); 
      palm_imu_gx = (int) (_gx * GYRO_TO_INT);
      palm_imu_gy = (int) (_gy * GYRO_TO_INT);
      palm_imu_gz = (int) (_gz * GYRO_TO_INT);
      t_last_palm_imu = micros();
    }
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration( _ax, _ay, _az );
      palm_imu_ax = (int) (_ax * ACC_TO_INT);
      palm_imu_ay = (int) (_ay * ACC_TO_INT);
      palm_imu_az = (int) (_az * ACC_TO_INT);
      t_last_palm_imu = micros();
    }
    // send data for gyro 0 (palm)
    sprintf( print_buffer, "G %d %d 0 accel %d %d %d gyro %d %d %d \n",
                 (seq_num++) & 0xff, 
                 millis() & 0xff, 
                 palm_imu_ax, palm_imu_ay, palm_imu_az, palm_imu_gx, palm_imu_gy, palm_imu_gz );
    Serial.print( print_buffer ); Serial.flush();
  }

  int t5 = micros();

  if (verbose > 3) {
    sprintf( print_buffer, "... sendAccelGyrotook %d usec.\n", (t5-t0) );
    Serial.print( print_buffer ); Serial.flush();
  }

}

void setAccelGyroOffsets( int* accelGyroOffsets ) 
{
  if (!have_lf_imu) return;

    sprintf( print_buffer, "... setAccelGyroOffsets: accel %d %d %d   gyro %d %d %d\n",
                  accelGyroOffsets[0],
                  accelGyroOffsets[1],
                  accelGyroOffsets[2],
                  accelGyroOffsets[3],
                  accelGyroOffsets[4],
                  accelGyroOffsets[5] );
    Serial.print( print_buffer ); Serial.flush();

  
    // use the code below to change accel/gyro offset values
    // accelgyro0.setXAccelOffset( accelGyroOffsets[0] );
    // accelgyro0.setYAccelOffset( accelGyroOffsets[1] );
    // accelgyro0.setZAccelOffset( accelGyroOffsets[2] );
   //  
    // accelgyro0.setXGyroOffset( accelGyroOffsets[3] );
    // accelgyro0.setYGyroOffset( accelGyroOffsets[4] );
    // accelgyro0.setZGyroOffset( accelGyroOffsets[5] );
}
  


/**
 * setup and initialize the HX711 strain gage (load cell) differential ADCs (if any)
 */
void init_HX711() 
{
  Serial.println( "... init_HX711..." );
  
  if (!have_hx711) {
    Serial.println( "... have_hx711 is globally DISABLED, skipping..." );
    return;
  }

  for (int i=0; i < N_LOADCELLS; i++ ) {
    sprintf( print_buffer, "... initializing loadcell %d data pin %d clk pin %d...",
                   i, loadcell_data_pins[i], loadcell_clk_pins[i] );
    Serial.println( print_buffer ); Serial.flush();
    
    loadcells[i].begin( loadcell_data_pins[i], loadcell_clk_pins[i] );
    loadcell_data[i] = loadcells[i].read(); // get a first reading
    loadcell_last_read_millis[i] = millis();

    // loadcells[i].tare(); // reset the scale to 0
    Serial.println( "... ok" );
  }
  
} // init_HX711()


void init_LSM6DS3()
{
  Serial.println( "... init_LSM6DS3..." );
  if (have_palm_imu) {
    int status = IMU.begin(); // library hardcodes the instance name...
    if (!status) have_palm_imu = false;
  }
}



void removeFirstCommandFromCommandBuffer() {
  bool vrb = (verbose > 6);
  if (vrb) Serial.print( "... removeFirstCommandFromCommandBuffer:\n" );
  if (vrb) {
    Serial.print( "... '" ); Serial.print( serialCommandBuffer ); Serial.println( "'" );
  }

  int ix = serialCommandBuffer.indexOf( '\n' );

  if (vrb) { Serial.print( "... index of first newline is %d\n" ); Serial.println( ix ); }
  if (ix >= 0) {
    serialCommandBuffer = serialCommandBuffer.substring( ix+1 );
  }
  if (vrb) Serial.print( "... '" ); Serial.print( serialCommandBuffer ); Serial.println( "'" );
}


/* slow blinking indicates the search is over. */
void handle_command() 
{
  // commands are <char><int><newline>
  int val = 0;
  int n_tokens = 0;
  char c = ' ';

  while( Serial.available()>0 ) {


    while( Serial.available() > 0) {
      c = Serial.read();
      serialCommandBuffer += c;
      if (verbose > 6) {
          Serial.print( "*** buffer '" ); Serial.print( serialCommandBuffer ); Serial.print( "' ***\n" );
      }
      
      if ((c == 10 ) || (c == 13)) {
        if (verbose > 5) Serial.println( "*** got \\n end-of-line!\n" );
        break;
      }
    }

    if ((c != 10) && (c != 13)) {
      break;
    }

    char cmd = serialCommandBuffer[0];
    int len = serialCommandBuffer.length();

    if (verbose > 5) {
      sprintf( print_buffer, "### serialCommandBuffer length %d\n", len );
      Serial.print( print_buffer ); Serial.flush();      
    }

    if (len > 200) {
      Serial.print( "### serialCommandBuffer overflow? len " ); Serial.println( len );
      Serial.print( "### Dropping current command: %c\n" ); Serial.println( cmd );
      Serial.flush();
      removeFirstCommandFromCommandBuffer();
      return;
    }

    if (verbose > 2) { 
      sprintf( print_buffer, "*** processing command char %c\n", cmd ); 
      Serial.print( print_buffer ); Serial.flush();
    }


    
    switch( cmd ) {
      case '?': // print help
      case 'h': usage();
                serialCommandBuffer = "";
                break;

      case '@': // enable=1 / disable=0 echo mode...
                {
                  int value = 0;                  
                  n_tokens = sscanf( serialCommandBuffer.c_str(), "@ %d", &value );
                  if (n_tokens == 1) {
                    echo_mode = value & 0x1;                    
                    sprintf( print_buffer,  "### echo mode is '%s'\n", echo_mode ? "enabled" : "disabled" );
                    Serial.print( print_buffer ); Serial.flush();
                  }
                  else {
                    sprintf( print_buffer,  "### Unknown command '%s'\n", serialCommandBuffer.c_str() );
                    Serial.print( print_buffer ); Serial.flush();
                  }
                }
                delay( 1000 );
                removeFirstCommandFromCommandBuffer();
                return;
                
      case '#': // enable=1 / disable=0 servos
                { // first, check whether we have seven tokens
                  int a0, a1, a2, a3, a4, a5, a6;
                  n_tokens = sscanf( serialCommandBuffer.c_str(), "# %d", 
                                     &a0 );
                  if (n_tokens == 1) { // ok, all values are there
                    sprintf( print_buffer, "### Set servo enabled: %d \n", 
                                     a0 );
                    Serial.print( print_buffer ); Serial.flush();

                    servo_enabled[0] = a0;
                    
                    removeFirstCommandFromCommandBuffer();
                    break;
                  }
                  // else complain
                  sprintf( print_buffer,  "### Unknown command '%s'\n", serialCommandBuffer.c_str() );
                  Serial.print( print_buffer ); Serial.flush();
                  removeFirstCommandFromCommandBuffer();
                }
                break;

      case 'A': // set acceleration limit (SMS). TODO: determine valid range(s)...
                { // first, check whether we have seven tokens
                  int a0;

                  // one parameter to be used for all seven servos
                  n_tokens = sscanf( serialCommandBuffer.c_str(), "A %d", &a0 );
                  if (n_tokens == 1) {
                    sprintf( print_buffer, "### Set common SMS acceleration %d", a0 );
                    Serial.print( print_buffer ); Serial.flush();

                    servo_accels[0] = a0;
                    wpaccel = a0;

                    removeFirstCommandFromCommandBuffer();
                    break;
                  }

                  // neither one parameters: complain
                  sprintf( print_buffer, "### Unknown command '%s'\n", serialCommandBuffer.c_str() );
                  Serial.print( print_buffer ); Serial.flush();
                  removeFirstCommandFromCommandBuffer();
                }
                break;

      case 'B': // set speed
                { 
                  int v0;
                  n_tokens = sscanf( serialCommandBuffer.c_str(), "B %d", &v0 );
                  

                  // check for single argument 
                  //
                  n_tokens = sscanf( serialCommandBuffer.c_str(), "B %d", &v0 );
                  if (n_tokens == 1) {
                    sprintf( print_buffer, "### Set common SMS servo speeds %d\n", v0 );
                    Serial.print( print_buffer ); Serial.flush();
 
                    servo_speeds[0] = v0;
                    wpspeed = v0;
                  }
                  else {
                   sprintf( print_buffer, "Invalid command '%s'\n", serialCommandBuffer.c_str() );
                   Serial.print( print_buffer ); Serial.flush();
                  }
                }
                removeFirstCommandFromCommandBuffer();
                break;

      case 'C': // close fingers by given degrees
                Serial.println( "### C: close fingers..." );
                n_tokens = sscanf( serialCommandBuffer.c_str(), "C %d", &val );
                if (n_tokens == 1) {
                  sprintf( print_buffer, "Close fingers by %d degrees \n", val );
                  Serial.print( print_buffer ); Serial.flush();

                  for( int i=0; i < N_SERVOS; i++ ) {
                    if (servo_enabled[i] != 1) continue; // skip, don't update previous goal

                    if (servo_signs[i] == +1) {
                      joint_position_counts[i] = min( servo_flex_positions[i],
                                                      joint_position_counts[i] + val*DEG2COUNTS );
                    }
                    else {
                      joint_position_counts[i] = max( servo_flex_positions[i],
                                                      joint_position_counts[i] - val*DEG2COUNTS );
                    }
                  }
                  setJointPositionGoal( joint_position_counts );
                }
                else {
                   sprintf( print_buffer, "Invalid command ignored: '%s'\n", serialCommandBuffer.c_str() );
                   Serial.print( print_buffer ); Serial.flush();
                }
                removeFirstCommandFromCommandBuffer();
                break;

      case 'D': // enable demo motions
                Serial.print( "Demo enabled\n" );
                demo_enable = 1;
                removeFirstCommandFromCommandBuffer();
                break;


      case 'E': // servo-hand effort goal: 1 <ints> (servo torque units!)
                {
                int tmp[ N_SERVOS ];
                n_tokens = sscanf( serialCommandBuffer.c_str(), "E %d", 
                                   &tmp[0] );
                if (n_tokens == 1) {
                  sprintf( print_buffer, "Set Effort/torque goal %d\n",
                                 tmp[0] );
                  Serial.print( print_buffer ); Serial.flush();

                  Serial.println( "*** IMPLEMENT ME!!! ***" );
                }
                else {
                   sprintf( print_buffer, "Unknown command >%s<\n", serialCommandBuffer.c_str() );
                   Serial.print( print_buffer ); Serial.flush();

                }
                removeFirstCommandFromCommandBuffer();
                }
                break;

      case 'F': // freeze: set new servo setpoint to current position
                sprintf( print_buffer, "*** Freeze command..." );

                Serial.print( print_buffer ); Serial.flush();

                setJointPositionGoal( joint_position_counts );
                removeFirstCommandFromCommandBuffer();
                break;

      case 'G': // servo-gripper postion goal: <int> (servo position counts!)
                {
                int tmp[ N_SERVOS ];
                n_tokens = sscanf( serialCommandBuffer.c_str(), "G %d", &tmp[0] );
                if (n_tokens == 1) {
                  setJointPositionGoal( tmp );
                }
                else {
                  sprintf( print_buffer, "### wrong number of tokens %d (req'd 1):\n", n_tokens );
                  Serial.print( print_buffer ); Serial.flush();
                }
                removeFirstCommandFromCommandBuffer();
                }
                break;

      case 'H': // move "home" - all stretched
                for( int j=0; j < N_JOINTS; j++ ) {
                  joint_position_goals[j] = servo_zero_positions[j];
                }
                setJointPositionGoal( joint_position_goals );
                removeFirstCommandFromCommandBuffer();
                break;

      case 'O': // open fingers by given degrees
                Serial.println( "### O: close fingers..." );
                n_tokens = sscanf( serialCommandBuffer.c_str(), "O %d", &val );
                if (n_tokens == 1) {
                  sprintf( print_buffer, "Open fingers by %d degrees \n", val );
                  Serial.print( print_buffer ); Serial.flush();

                  for( int i=0; i < N_SERVOS; i++ ) {
                    if (servo_signs[i] == +1) {
                      joint_position_counts[i] = max( servo_open_positions[i],
                                                      joint_position_counts[i] - val*DEG2COUNTS );
                    }
                    else {
                      joint_position_counts[i] = min( servo_open_positions[i],
                                                      joint_position_counts[i] + val*DEG2COUNTS );
                    }
                  }
                  setJointPositionGoal( joint_position_counts );
                }  
                else {
                  sprintf( print_buffer, "Invalid command ignored: >>%s<<\n", serialCommandBuffer.c_str() );
                  Serial.print( print_buffer ); Serial.flush();
                }
                removeFirstCommandFromCommandBuffer();
                break;

      case 'Q': // move to given position (selected SMS servo) <pos> or <pos> <speed> <acc>
                {
                  int vv, aa;
                  n_tokens = sscanf( serialCommandBuffer.c_str(), "Q %d %d %d", &val, &vv, &aa );
                  if (n_tokens == 3) {
                    sprintf( print_buffer, "Write SMS pos %d vel %d acc %d\n", val, vv, aa );
                    Serial.print( print_buffer ); Serial.flush();
                    sms.WritePosEx( sid, val, vv, aa ); // id, position, speed, accel
                    removeFirstCommandFromCommandBuffer();
                    Serial.println( "... QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQq" );
                    break;
                  }
                  
                  n_tokens = sscanf( serialCommandBuffer.c_str(), "Q %d", &val );
                  if (n_tokens == 1) {
                    sprintf( print_buffer, "Write pos %d\n", val );
                    Serial.print( print_buffer ); Serial.flush();
                    sms.WritePosEx( sid, val, wpspeed, wpaccel ); // id, position, global speed, accel
                    Serial.println( "... QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQq" );
                  }
                  else {
                    sprintf( print_buffer, "Unknown command '%s'\n", serialCommandBuffer.c_str() );
                    Serial.print( print_buffer ); Serial.flush();
                    Serial.println( "QQQQQQQ \n\n\n\n\n\n\n" );
                  }
                  removeFirstCommandFromCommandBuffer();
                  break;
                }
      case 'R': // write register <id> <addr> <value>
                int servoID, regAddr, regValue;
                n_tokens = sscanf( serialCommandBuffer.c_str(), "R %d %d %d", &servoID, &regAddr, &regValue );
                if (n_tokens == 3) {
                  sprintf( print_buffer, "### Set SMS register: servo %d register %d value %d\n",
                                 servoID, regAddr, regValue );
                  Serial.print( print_buffer ); Serial.flush();

                  sms.writeByte( servoID, regAddr, regValue );
                }
                else {
                  sprintf( print_buffer, "Unknown command '%s'\n", serialCommandBuffer.c_str() );
                  Serial.print( print_buffer ); Serial.flush();
                }
                removeFirstCommandFromCommandBuffer();
                break;

      case 'T': // torque limits: 1 <ints> (servo position counts!)
                {
                int tmp[ N_SERVOS ];
                n_tokens = sscanf( serialCommandBuffer.c_str(), "T %d", &tmp[0] );
                if (n_tokens == 1) {
                  setServoTorqueLimits( tmp );
                }
                else {
                  sprintf( print_buffer, "### wrong number of tokens %d (req'd 1):\n", n_tokens );
                  sprintf( print_buffer, "### command was %s'\n'", serialCommandBuffer.c_str() );
                  Serial.print( print_buffer ); Serial.flush();
                }
                removeFirstCommandFromCommandBuffer();
                }
                break;


      case 'Y': // set verbose level <int>
                {
                  int level = 0;
                  n_tokens = sscanf( serialCommandBuffer.c_str(), "Y %d", &level );
                  if (n_tokens == 1) {
                    sprintf( print_buffer, "New debug/verbose level %d\n", level );
                    Serial.print( print_buffer ); Serial.flush();
                    verbose = level;
                  }
                  else {
                    sprintf( print_buffer, "Invalid Y command '%s'\n", serialCommandBuffer.c_str() );
                    Serial.print( print_buffer ); Serial.flush();
                  }
                }
                removeFirstCommandFromCommandBuffer();
                break;

      case (int) 27: // Ascii Escape
                demo_enable = 0;
                freeze_now();
                Serial.print( "Demo disabled\n" );
                removeFirstCommandFromCommandBuffer();
                break;                

      case 'd': // disable demos
                demo_enable = 0;
                removeFirstCommandFromCommandBuffer();
                break;

      case 'f': // print strain-gage forces
                Serial.println( "Print FORCES NOT IMPLEMENTED" );
                removeFirstCommandFromCommandBuffer();
                break;                

      case 's': // select servo
                n_tokens = sscanf( serialCommandBuffer.c_str(), "s %d", &val );
                if (n_tokens == 1) {
                  sprintf( print_buffer, "selected servo %d (%x)\n", sid, sid );
                  Serial.print( print_buffer ); Serial.flush();
                  if ((val >= 3) && (val <= 253)) sid = val;
                }
                else {
                  sprintf( print_buffer, "Select servo failed: '%s'\n", serialCommandBuffer.c_str() );
                  Serial.print( print_buffer ); Serial.flush();
                }
                removeFirstCommandFromCommandBuffer();
                break;
                
      case 'i': // print current (SCS)
                sprintf( print_buffer, "current %d\n", scs.ReadCurrent( sid ) );
                Serial.print( print_buffer ); Serial.flush();
                removeFirstCommandFromCommandBuffer();
                break;

      case 'j': // print current (SMS)
                sprintf( print_buffer, "current %d\n", sms.ReadCurrent( sid ) );
                Serial.print( print_buffer ); Serial.flush();
                removeFirstCommandFromCommandBuffer();
                break;
                
      case 'l': // print load (torque) (SCS)
                sprintf( print_buffer, "torque %d\n", scs.ReadLoad( sid ) );
                Serial.print( print_buffer ); Serial.flush();
                removeFirstCommandFromCommandBuffer();
                break;

      case 'm': // print load (torque) (SMS)
                sprintf( print_buffer, "torque %d\n", sms.ReadLoad( sid ) );
                Serial.print( print_buffer ); Serial.flush();
                removeFirstCommandFromCommandBuffer();
                break;

      case 'p': // print current position (SCS)
                sprintf( print_buffer, "position %d\n", scs.ReadPos( sid ) );
                Serial.print( print_buffer ); Serial.flush();
                removeFirstCommandFromCommandBuffer();
                break;

      case 'q': // print current position (SMS) (changed byte order!)
                sprintf( print_buffer, "position %d\n", sms.ReadPos( sid ) );
                Serial.print( print_buffer ); Serial.flush();
                removeFirstCommandFromCommandBuffer();
                break;

      case 'r': // read servo byte register <id> <addr>, output written to serial
                {
                int servoID, regAddr, regValue;
                n_tokens = sscanf( serialCommandBuffer.c_str(), "r %d %d", &servoID, &regAddr );
                if (n_tokens == 2) {
                  regValue = sms.readByte( servoID, (u8) regAddr);
                  sprintf( print_buffer, "### Read SCS register: servo %d register %d value %d\n",
                                 servoID, regAddr, regValue );
                  Serial.print( print_buffer ); Serial.flush();
                }
                else {
                  sprintf( print_buffer, "Unknown/invalid command '%s'\n", serialCommandBuffer.c_str() );
                  Serial.print( print_buffer ); Serial.flush();
                }
                removeFirstCommandFromCommandBuffer();
                break;
                sprintf( print_buffer, "position %d\n", sms.ReadPos( sid ) );
                Serial.print( print_buffer ); Serial.flush();
                removeFirstCommandFromCommandBuffer();
                }
                break;

      case 't': // print current temperature
                sprintf( print_buffer, "temp.    %d\n", scs.ReadTemper( sid ) );
                Serial.print( print_buffer ); Serial.flush();
                removeFirstCommandFromCommandBuffer();
                break;

      case 'v': // print voltage
                sprintf( print_buffer, "voltage %d\n", scs.ReadVoltage( sid ) );
                Serial.print( print_buffer ); Serial.flush();
                removeFirstCommandFromCommandBuffer();
                break;

      case 'w': // write servo byte register <id> <addr> <value>, value also read-back and written to serial
                {
                  int servoID, regAddr, regValue, newValue, status;
                  n_tokens = sscanf( serialCommandBuffer.c_str(), "w %d %d %d", &servoID, &regAddr, &regValue );
                  if (n_tokens == 3) {
                    status   = sms.writeByte( servoID, (u8) regAddr, (u8) regValue );
                    if (status == -1) {
                      sprintf( print_buffer, "### Write SCS register: servo %d register %d value %d failed: status %d.",
                                     servoID, regAddr, regValue, status );
                      Serial.print( print_buffer ); Serial.flush();
                    }
                    else { // good so far, read newly written value back immediately
                      newValue = sms.readByte( servoID, (u8) regAddr );
                      sprintf( print_buffer, "### Write SCS register: servo %d register %d value %d read-back %d\n",
                                     servoID, regAddr, regValue, newValue );
                      Serial.print( print_buffer ); Serial.flush();

                    }
                  }
                  else {
                    sprintf( print_buffer, "Unknown/invalid command '%s'\n", serialCommandBuffer.c_str() );
                    Serial.print( print_buffer ); Serial.flush();
                  }
                  Serial.flush();
                  removeFirstCommandFromCommandBuffer();
                }
                break;

      case '\n': // do nothing;
                removeFirstCommandFromCommandBuffer();
                break;

      default:  // delete unknown/unsupported characters immediately
                sprintf( print_buffer, "ignored command '%c'\n", c );          
                Serial.print( print_buffer ); Serial.flush();

                serialCommandBuffer = "";
                break;
                
    }
  }
} // handle_command







void setup()
{
  pinMode( LED_PIN, OUTPUT );
  digitalWrite( LED_PIN, HIGH );

  enableDebugMsgs = (verbose > 5);
  
  // Serial.begin(115200);
  Serial.begin( 230400 );
  while( !Serial ) { // fast blinking until we have a serial console
    delay( 100 );
    digitalWrite( LED_PIN, LOW );
    delay( 100 );
    digitalWrite( LED_PIN, HIGH );
  }

  // print welcome and "usage" message first
  //
  usage();

  // detect and initialize the Feetech SCS servos. 
  // Note: we may have different servo types on same bus, same serial port, baudrate default is 1Mbaud
  // 
  Serial.println( "... Searching SCS/SMS servos..., wait until loop completes..." );
  initSCS();

  // initialize HX711 differential ADCs (if any)
  init_HX711();

  // initialize I2C bus and MPU6050 IMUs (if any)
  init_LSM6DS3();
 
  Serial.println( "setup() ok, starting loop()..." );
} // end setup()




void loop() {
  iteration ++;
  if ( (iteration < 5) || (iteration == 99) || ((iteration & 0x07ff) == 0x07ff)) {
    sprintf( print_buffer, "... iteration %d\n", iteration ); Serial.flush();
    Serial.print( print_buffer ); Serial.flush();
  }
  
  long t_now = millis();
  
  if ((t_now - t_blink) > 1000) {
    led_val = led_val ^ 1;
    digitalWrite( LED_PIN, led_val );  
    t_blink = t_now;
  }

  // these functions run every loop cycle
  sendServoPositions();
  sendServoTorques();
  sendServoPositionErrors();
  // sendJointVelocities(); // host calculates from pos-diffs

  // every loop cycle, sensor sampling may be auto-throttled
  sendStrainGageForces();

  // every loop cycle, sensor sampling may be auto-throttled
  sendAccelGyro();

  // these functions auto-throttle themselves
  sendServoTemperatures();
  sendServoVoltages();
  sendServoCurrents();


  // check for incoming user/host commands
  if (Serial.available()) {
    handle_command();
  }

  // want to run at ~100 Hz
  long t_now2      = millis();
  long loopMillis  = t_now2 - t_now;
  long sleepMillis = max( 0, 10 - loopMillis );
  if ((verbose > 4) && (loopMillis < 10)) {
    sprintf( print_buffer, "*** loop took %d msec, need extra sleep %d msec...\n", 
                   loopMillis, sleepMillis );
    Serial.print( print_buffer ); Serial.flush();
  }
  delay( sleepMillis );
  // delay( 500 ); // 100 
}
