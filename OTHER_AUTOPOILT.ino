/*

differential drive motorboat autopilot, using GPS & BNO055 9DOF sensor

This version decodes $GPGGA sentences to get position and number of satellites in use

LV168 @ 20 MHz  Atmel Studio IV/ avr-gcc

sjr  7-9/2015

build options for floating point output:
add -Wl,-u,vfprintf to the LINKER options. It appears to only cater to printf but not scanf.
add -lprintf_flt or -lscanf_flt to the linker options which is done by including printf_flt as a library.
need to set -lm if using float in any case
compiler.c.elf.extra_flags= -Wl,-u,vfprintf -lprintf_flt -lm   -Wl,-u,vfscanf -lscanf_flt -lm

*/
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include <util/delay.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>

// serial I/O on UART



// set up uart as STDOUT

static FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

// basic I2C routines
#include "I2C.c"

// BNO055 I2C extras
#include "bno055_I2C.c"

// GPS routines
#include "gps_gga.c"

#define gps_parse_enable() gps_parse_enable=1

// routine to calculate heading error in degrees, taking into account compass wrap

int heading_error(int bearing, int current_heading)
{
 int error = current_heading - bearing;
// if (error >  180) error -= 360;
// if (error < -180) error += 360;
 error = (error + 540)%360-180;
 return error;
}

// routine to correct for compass wrap

int wrap360(int direction) {
//  while (direction > 359) direction -= 360;
//  while (direction <   0) direction += 360;
    direction=(direction+360)%360;
  return direction;
}


int main( void ) {

  unsigned char result;
  int i,v[3];

//predetermined BNO055 calibration constants

  int precal[]={-6,3,17,-2,395,112,0,1,0,1000,832}; //averages of 9 in situ calibration runs
  int motorSpeed=150, kp=40, bias=0, error=0;

  unsigned long timeout=50000UL; //max run time per leg in milliseconds
  unsigned long gps_last_fix=0;  //time since start of last GPS fix
  unsigned long gps_timeout=5000UL; //5 seconds since last fix

  int leg=0;  //current course leg
  int start_bearing = 0, point_bearing=0, gps_bearing=0, imu_heading=0;

  int yaw_offset = 13; //calibration factor to correct compass readings to true North

    double leg_dist=20.0, distance=0.0;
  int dist_to_go=0;

    long lat=0,lon=0;   //where we are now
  long lat1=0,lon1=0; //course start
  long lat2=0,lon2=0; //current destination

  int num_sats=0;

//  setup

    gps_parse_init(9600); //enable uart

  stdout = &uart_str; //standard out for printf

  delay(1000); //wait for BNO055 power up

  I2C_Init();

// check for BNO055 presence

  if (0xa0 != (result=I2C_ReadRegister(BNO055_A0,BNO055_WHO_AM_I))) { //check internal device ID
    clear();
    print(" No BNO");
    while(1); //hang here
    }

// reset BNO055, in case of soft reboot
  I2C_WriteRegister(BNO055_A0, BNO055_SYS_TRIGGER, 0x20); //BNO055 system reset
  delay(1000); //required reset delay

// store earlier derived calibration parameters
  I2C_WriteCal(BNO055_A0, BNO055_CAL_DATA, (int *)precal);
  I2C_WriteRegister(BNO055_A0, BNO055_SYS_TRIGGER, 0x80); //use external 32 kHz crystal.

// set to measurement mode

  I2C_WriteRegister(BNO055_A0,BNO055_OPER_MODE,BNO055_OPER_MODE_NDOF);
  delay(10); //minimum 7 ms delay

//  end of setup. Now,
// get/set default initial heading using compass

  sei(); //enable interrupts
  gps_parse_status = 0; //ignore old results
  gps_parse_enable(); //(re)enable parsing
  lcd_goto_xy(0,1);
  print(" no fix");

// wait till GPS has fix

  while (gps_parse_status != 1);
  gps_parse_enable();

  clear();
  print("Aim");
  lcd_goto_xy(0,1);
  print("Push B");
  delay(1000);

// get compass reading, wait for button press

  unsigned long timer=millis();

  do {
    I2C_Read3Vectors(BNO055_A0,BNO055_FUSED_EULER, (int16_t *)v);

    if(gps_parse_status == 1) {
    num_sats = gga_num_sats();
    gps_parse_enable(); //(re)enable parsing
    }

    clear();
    imu_heading = wrap360((v[0]>>4) + yaw_offset);
    print_long(imu_heading); //yaw
    lcd_goto_xy(1,1);
    print_long(num_sats);
    print("s ");
    delay(300);
   }

   while (button_is_pressed(ANY_BUTTON)==0);  //HANG HERE until button pressed

// wait for next GPS fix

   while (gps_parse_status != 1);

// start the run

  gps_last_fix = millis();
  gps_latlon(&lat1, &lon1); //record start point in lat1, lon1
  gps_parse_enable();

  point_bearing = imu_heading; //this is where we will go
  start_bearing = point_bearing; //save it for calculating next leg

// project new waypoint "distance" meters ahead -> (lat2, lon2)

  bearing_shoot((float)start_bearing, leg_dist, lat1, lon1, &lat2, &lon2);

// check!

  gps_bearing = (int) (course_to(lat1, lon1, lat2, lon2, &distance)+0.5);
  dist_to_go = distance + 0.5;

// start telemetry

  printf("S,%lu,%ld,%ld,%ld,%ld,%d,%d\n",millis(),lat1,lon1,lat2,lon2,gps_bearing,dist_to_go);

// start the course leg clock

  timer = millis();

// run course

  while(2) {

        switch (gps_parse_status)
    {

    case 0: break;  //still decoding

    case 1:  //good fix
      {
      gps_last_fix = millis();
      gps_latlon(&lat, &lon);
      gps_parse_enable(); //re-enable parsing

      gps_bearing = (int) (course_to(lat, lon, lat2, lon2, &distance)+0.5);
      dist_to_go = distance + 0.5;

// To take into account wind and currents,
// substitute calculated GPS bearing for point_bearing

      point_bearing = gps_bearing;
      num_sats = gga_num_sats();

// telemetry
       printf("G,%lu,%ld,%ld,%d,%d,%d\n",millis(),lat,lon,gps_bearing,dist_to_go,num_sats);
             }
       break;

     case 2:  //no fix
        {
      clear();
      print(" no fix");
      printf("E,%lu,2,0,0,0,0\n",millis()); //error dump
      break;
      }

    case 3:  //chksum error
      {
        clear();
      print("?chk");
      printf("E,%lu,3,0,0,0,0\n",millis()); //error dump
      break;
      }

    default:  //shouldn't happen
      {
      clear();
      print("-E-");
      printf("E,%lu,4,0,0,0,0\n",millis()); //error dump
      }
    } //end switch

// continue to run this leg, steering with IMU
// (may not have recent GPS info in the following)

    I2C_Read3Vectors(BNO055_A0,BNO055_FUSED_EULER, (int16_t *)v);  //get Euler angles
    result=I2C_ReadRegister(BNO055_A0,BNO055_CALIB_STAT); //check calibration status

    for(i=0; i<3; i++) v[i] >>= 4;
    imu_heading = wrap360(v[0] + yaw_offset);

// update LCD display
    clear();
    print_long(point_bearing);
    print(" ");
    print_long(imu_heading);
    lcd_goto_xy(1,1);
    print_long(num_sats);
    print("s ");
    print_long(dist_to_go);

// heading error and PID steering

    error = heading_error(point_bearing, imu_heading);

    //error is positive if current_heading > bearing (compass direction)
    //positive bias acts to reduce left motor speed, so bear left

    bias = (kp*error)/10;  //Kp in tenths (Ki, Kd not necessary in water)

    printf("I,%lu,%d,%d,%d,%d,%d,%d,%u,\n",millis(),
    point_bearing,imu_heading,error,bias,v[1],v[2],result);

    // the motor routines internally limit the argument to {-255, 255}

    set_m1_speed(motorSpeed-bias); //left motor
    set_m2_speed(motorSpeed+bias); //right motor

// check for recent GPS fix 
// 2024 september 6, do you remeber me?

    if (millis() - gps_last_fix > gps_timeout) //5 seconds, currently
      {
      set_motors(0,0); //stop motors
      clear();
      print(" GPS-to"); //timeout!
      printf("E,%lu,5,0,0,0,0\n",millis()); //error dump
      delay(300);//wait a bit
      }
    else {
      if ((millis()-timer) > timeout || dist_to_go < 3)
      {

// leg timeout or at goal, stop motors

      clear();
      if (dist_to_go < 3) print("DIST");
      else print("TIME");
      set_motors(0,0);
      delay(1000); //drift a bit

      leg++;  //count course legs accomplished

// course is an equilateral triangle. Compute next goal

      start_bearing = wrap360(start_bearing+120);

      // calculate coordinates of next target, from where we should be

      if (leg==1) {
         bearing_shoot((float)start_bearing, leg_dist, lat2, lon2, &lat2, &lon2);
         }

      if (leg == 2) {
         lat2=lat1; //third leg, set destination coords to
         lon2=lon1; //start point
         }

      if (leg == 3) break; //done

      //telemetry
      printf("S,%lu,%ld,%ld,%ld,%ld\n",millis(),lat,lon,lat2,lon2);

      // recalculate course from where we are now to current destination

      point_bearing = (int) (course_to(lat, lon, lat2, lon2, &distance) + 0.5);
      dist_to_go = distance + 0.5;
      timer=millis();
      } //if(timeout)
    } //if gps_timeout else
  }  //while(2)

  clear();
  print("stop");
  char cz=26;
  printf("%c%c%c",cz,cz,cz); //end log file
  while(1); //hang
  return 0;
} //main
