/**
 * MyBot I2C robot pathfinder2
 *
 * Scan command:  [STX][CMD][VAL1][ETX][CRC]
 * Example:        0xFC 0x03 0xA4 0x00 0xFE 0x93
 *
 *    VAL1: Scan type
 *
 * Drive command: [STX][CMD][VAL1][VAL2][VAL3][ETX][CRC]
 * Example:       0xFC 0x03 0xA4 0x00 0x00 0xFE 0x93
 *
 *    VAL1: Speed to drive 1-100
 *    VAL2: PWM Frequence 2000 - 5000
 *
 * We are going to add something for Git to detect..
 *
 */

#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>
#include <errno.h>
#include <stdbool.h>
#include <gpiod.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "servo.h"
#include "pathfinder.h"
#include "infrared.h"
#include "lcd.h"
#include "gyro.h"

int i2cFile;

bool bDebug = false;
bool agtDebug = false;
bool driveDebug = false;
bool i2cDevOpen = false;
bool driveForward = false;
bool driveOk = true;
bool runDrive = true;
bool useLidar = true;
bool irValid = false;

uint8_t rpsCheckCount = 0;
int sharedMemId = 0;
char errorBuf[120];
uint8_t replyBuffer[BUFFER_SIZE];
uint8_t requestBuffer[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t scanData[SCAN_BUFFER_SIZE];

char *bon = "1";
char *boff = "0";

char *currentI2CDevice = NULL;
char *driveCtrl = "Drive Controller:";
char *scanCtrl = "Scan Controller:";
char *gyroCtrl = "Gyro Controller";
char *lcdCtrl = "LCD Controller";
char *adCtrl = "A To D Controller";
char *i2cDevice = "/dev/i2c-2";

void send_lcd_msg(char *msg);
bool checkForwardLidar();
bool checkRightLidar();
bool checkForwardRightLidar();
bool checkBackwardLidar();
bool checkLeftLidar();
bool checkForwardLeftLidar();
bool getSharedMemory(int id);

#include "standard_functions.c"
#include "lcd.c"
#include "infrared_functions.c"

/*
   Send command to turn on/off debugging code on the scanner
   controller Pico
*/
void turn_on_scan_debug()
{
   if (bDebug)
   {
      printf("turn_on_scan_debug()\n");
      fflush(stdout);
   }
   requestBuffer[0] = C_STX;
   requestBuffer[1] = C_SCAN_DATA_DEBUG;
   requestBuffer[2] = 0x00;
   requestBuffer[3] = C_ETX;
   requestBuffer[4] = gencrc(requestBuffer, 4);
   send_request(5, 1);
}

/*
   Simple function to print scan data
*/
void print_scan_data()
{
   printf("Right: %d inches\n", scanData[0]);
   printf("Left: %d inches\n", scanData[1]);
   printf("Forward: %d inches\n", scanData[2]);
   printf("Backward: %d inches\n", scanData[3]);
}

/*
   Send command to retrieve all scan data from the scan
   controller Pico
*/
bool get_us_scan_data()
{
   bool rs = false;

   if (bDebug)
   {
      printf("get_full_scan_data()\n");
      fflush(stdout);
   }
   if (set_ioctl_addr(SCAN_I2C_ADDR))
   {
      requestBuffer[0] = C_STX;
      requestBuffer[1] = C_SCAN_ALL;
      requestBuffer[2] = 0x00;
      requestBuffer[3] = C_ETX;
      requestBuffer[4] = gencrc(requestBuffer, 4);
      if (send_request(5, 4))
      {
         for (int i = 0; i < 4; ++i)
         {
            scanData[i] = replyBuffer[i];
            switch(i) {
               case RIGHT_DATA_BUF:
               if (scanData[i] > MIN_SIDE_DISTANCE)
                  send_status_msg(bon, DB_STATUS_LINE, DB_US_RIGHT);
               else
                  send_status_msg(boff, DB_STATUS_LINE, DB_US_RIGHT);
                  break;
               case LEFT_DATA_BUF:
               if (scanData[i] > MIN_SIDE_DISTANCE)
                  send_status_msg(bon, DB_STATUS_LINE, DB_US_LEFT);
               else
                  send_status_msg(boff, DB_STATUS_LINE, DB_US_LEFT);
                  break;
               case FRONT_DATA_BUF:
               if (scanData[i] > MIN_FORWARD_DISTANCE)
                  send_status_msg(bon, DB_STATUS_LINE, DB_US_FRONT);
               else
                  send_status_msg(boff, DB_STATUS_LINE, DB_US_FRONT);
                  break;
               case BACK_DATA_BUF:
               if (scanData[i] > MIN_BACKWARD_DISTANCE)
                  send_status_msg(bon, DB_STATUS_LINE, DB_US_BACK);
               else
                  send_status_msg(boff, DB_STATUS_LINE, DB_US_BACK);
                  break;
            }
         }
         rs = true;
      }
   }
   return rs;
}

#include "drive_functions.c"
#include "gyro.c"
#include "servo.c"

/*
   Check the PIR motion sensor to see if there is anthing
   moving.  Not really implimented at this time.
*/
bool check_motion_sensor()
{
   bool rs = false;

   if (bDebug)
   {
      printf("check_motion_sensor()\n");
      fflush(stdout);
   }
   if (set_ioctl_addr(DRIVER_I2C_ADDR))
   {
      requestBuffer[0] = C_STX;
      requestBuffer[1] = C_DRIVE_MTN;
      requestBuffer[2] = 0x00;
      requestBuffer[3] = 0x00;
      requestBuffer[4] = C_FREQ_MULTIPLIER;
      requestBuffer[5] = C_ETX;
      requestBuffer[6] = gencrc(requestBuffer, 6);
      if (send_request(7, 1))
      {
         if (replyBuffer[0] == 1)
         {
            if (bDebug)
            {
               printf("Motion detected: %d\n", replyBuffer[0]);
            }
            rs = true;
         }
      }
   }

   return rs;
}

/*
   Get the revolutions per second from the drive Pico
   controller.
*/
bool check_rps()
{
   bool rs = false;

   if (bDebug)
   {
      printf("Function check_rps()\n");
   }
   if (set_ioctl_addr(DRIVER_I2C_ADDR))
   {
      requestBuffer[0] = C_STX;
      requestBuffer[1] = C_DRIVE_RPS;
      requestBuffer[2] = 0x00;
      requestBuffer[3] = 0x00;
      requestBuffer[4] = C_FREQ_MULTIPLIER;
      requestBuffer[5] = C_ETX;
      requestBuffer[6] = gencrc(requestBuffer, 6);
      if (send_request(7, 1))
      {
         if (replyBuffer[0] == 0)
         {
            ++rpsCheckCount;
            if (rpsCheckCount > 1)
            {
               rpsCheckCount = 0;
               rs = true;
               if (driveDebug)
               {
                  printf("Forward motion not detected: %d\n", replyBuffer[0]);
               }
            }
         }
      }
      else
      {
         rs = true;
         if (driveDebug)
         {
            printf("Error sending C_DRIVE_RPS request!\n");
         }
      }
   }
   else
   {
      rs = true;
      if (driveDebug)
      {
         printf("Error set_ioctl_addr for DRIVER_I2C_ADDR:%d!\n", DRIVER_I2C_ADDR);
      }
   }
   return rs;
}

#include "lidar_functions.c"

/*
   Capture any interrupt signal received and make sure that
   the drive motors are stopped.
*/
void signal_callback_handler(int signum)
{
   char msg[16];

   drive_stop();
   sprintf(msg, "Caught signal %d", signum);
   perror(msg);
   send_lcd_msg(msg);
   sleep(1);
   //lcd_off();
   drive_stop(); // repeat
   removeSharedMemory(sharedMemId);
   // driveOk = false;
   exit(signum);
}

int main(int argc, char *argv[])
{
   int driveTime = 0;
   time_t randTime;
   float ads1114_val = 0.0;
   int lidar_loop = 0;
   bool usScanOk = false;

   srand((unsigned)time(&randTime));
   signal(SIGINT, signal_callback_handler);

   if (argc == 2)
   {
      printf("The argument is %s\n", argv[1]);
      uint8_t cmd = (uint8_t)atoi(argv[1]);
      if (cmd == 1)
         bDebug = true;
      else if (cmd == 2)
         agtDebug = true;
      else if (cmd == 3)
         driveDebug = true;
      else if (cmd == 4)
      {
         bDebug = true;
         agtDebug = true;
         driveDebug = true;
      }
   }

   open_i2c_device();
   irValid = init_ir_device();
   init_accel_gyro_temp();
   get_accel_gyro_temp();
   //init_lcd_device();

   sharedMemId = setSharedMemory(KEY);
   if (sharedMemId == -1)
      useLidar = false;
   else
      printSharedMemoryInfo(sharedMemId);

   char tmsg[64] = "Pathfinder:";
   send_lcd_msg(tmsg);
   //   sleep(3);
   //   i2cDevOpen = false;

   if (i2cDevOpen)
   {
      // Time to scan for obstacles
      rpsCheckCount = 0;
      lidar_loop = 0;
      useLidar = getSharedMemory(sharedMemId);
      if (useLidar == false && driveDebug == true)
      {
         perror("Lidar data shared memory not detected!  Revert to ultrasonic sensors.\n");
      }
      while (driveOk)
      {
         ads1114_val = get_ads1115_value(IR_READ_A0);
         ++lidar_loop;
         if (lidar_loop > 2)
         {
            lidar_loop = 0;
            useLidar = getSharedMemory(sharedMemId);
         }
         if (runDrive)
         {
            get_accel_gyro_temp();
            //check_motion_sensor();
            usScanOk = get_us_scan_data();

            // If LIDAR is active refer to it first to drive the robot
            if (useLidar)
            {
               if (checkForwardLidar())
               {
                  // We are ok to go forward
                  driveForward = drive_forward();
               }
               else
               {
                  driveForward = drive_stop();
                  determine_turn(DT_CHECK_LIDAR);
               }
            }
            // Check the ultrasonic sensors next
            else if (usScanOk)
            {
               if (scanData[FRONT_DATA_BUF] > MIN_FORWARD_DISTANCE)
               {
                  if (driveForward == false)
                  {
                     driveForward = drive_forward();
                  }
               }
               else
               {
                  driveForward = drive_stop();
                  if (scanData[FRONT_DATA_BUF] < MIN_TURN_DISTANCE)
                  {
                     if (scanData[BACK_DATA_BUF] > BACKUP_DISTANCE)
                     {
                        drive_backup(SHORT_BACKUP_TIME);
                     }
                  }
                  determine_turn(DT_CHECK_US);
               }
            }
            // Check the forward infrared sensors next
            if (irValid)
            {
               if (check_ir_sensors())
               {
                  driveForward = drive_stop();
                  determine_turn(DT_CHECK_IR);
               }
            }
            // If we are driving forward check our front wheel encoder to make
            //    sure we have some forward motion.  If no forward motion is
            //    detected then stop and determine which direction to turn.
            if (driveForward)
            {
               if (check_rps())
               {
                  // we are not detecting any forward motion
                  driveForward = drive_stop();
                  if (useLidar)
                  {
                     if (checkBackwardLidar())
                     {
                        drive_backup(SHORT_BACKUP_TIME);
                        determine_turn(DT_CHECK_LIDAR);
                     }
                  }
                  else if (scanData[BACK_DATA_BUF] > BACKUP_DISTANCE)
                  {
                     drive_backup(SHORT_BACKUP_TIME);
                     determine_turn(DT_CHECK_US);
                  }
               }
            }
         }               // if runDrive
         usleep(100000); // 1/10 second
      }                  // driveOk loop
   }
   else
   {
      perror("I2C open error!\n");
   }
   drive_stop();
   removeSharedMemory(sharedMemId);
   return 0;
}

