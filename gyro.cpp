
/*
   Initialize the accelerator/gyroscope board
*/
bool init_accel_gyro_temp()
{
   if (set_ioctl_addr(MPU6050_I2C_ADDR)) {
      requestBuffer[0] = REG_PWR_MGMT_1;
      requestBuffer[1] = 0x01;
      send_request(2, 1);
      requestBuffer[0] = REG_ACCEL_CONFIG;
      requestBuffer[1] = 0x00;
      send_request(2, 1);
      requestBuffer[0] = REG_SMPRT_DIV;
      requestBuffer[1] = 0x07;
      send_request(2, 1);
      requestBuffer[0] = REG_CONFIG;
      requestBuffer[1] = 0x04;
      send_request(2, 1);
      requestBuffer[0] = REG_GYRO_CONFIG;
      requestBuffer[1] = 0x18;
      send_request(2, 1);
      requestBuffer[0] = REG_INT_ENABLE;
      requestBuffer[1] = 0x01;
      send_request(2, 1);
      //requestBuffer[0] = REG_FIFO_EN;
      //requestBuffer[1] = 0x88;
      //send_request(2, 1);
      //requestBuffer[0] = REG_USER_CTRL;
      //requestBuffer[1] = 0x44;
      //send_request(2, 1);
      return true;
   }
   return false;
}

/*
   Get the raw data from the accl/gyro board using the I2C
   bus.
*/
int16_t read_raw_data(char reg_address) {
   short high_byte,low_byte;
   int16_t value;
   char buf[1];

   if (agtDebug) {
      printf("%s: Sending - %02x\n", currentI2CDevice, reg_address);
   }
   if (set_ioctl_addr(MPU6050_I2C_ADDR)) {
      buf[0] = reg_address;
      if (write(i2cFile, buf, 1) != 1) {
         printf("Error, unable to write to i2c device %02x\n", buf[0]);
         return 0;
      }
      if (read(i2cFile, &high_byte, 1) != 1) {
         printf("Error, unable to read from i2c device %02x\n", buf[0]);
         return 0;
      }
      buf[0] = reg_address+1;
      if (write(i2cFile, buf, 1) != 1) {
         printf("Error, unable to write to i2c device %02x\n", buf[0]);
         return 0;
      }

      if (read(i2cFile, &low_byte, 1) != 1) {
         printf("Error, unable to read from i2c device %02x\n", buf[0]);
         return 0;
      }
      //value = (high_byte << 8) | low_byte;  
      value = two_complement_to_int(high_byte, low_byte);
      return value;
   }
   return 0;
}

float gAcc_x_h = 0.0;
float gAcc_y_h = 0.0;
float gGyro_x_h = 0.0;
float gGyro_y_h = 0.0;
float gAcc_x_l = 0.0;
float gAcc_y_l = 0.0;
float gGyro_x_l = 0.0;
float gGyro_y_l = 0.0;
bool gGyroPass = false;

/*
   Obtain the accelerator, gyroscope and temperature
   from the board.
*/
bool get_accel_gyro_temp()
{
	float Acc_x,Acc_y,Acc_z;
	float Gyro_x,Gyro_y,Gyro_z;
	float Ax=0, Ay=0, Az=0;
	float Gx=0, Gy=0, Gz=0;
   float Tt=0, TempF=0, TempC=0;

   if (set_ioctl_addr(MPU6050_I2C_ADDR)) {
		Acc_x = read_raw_data(REG_ACCEL_XOUT_H);
		Acc_y = read_raw_data(REG_ACCEL_YOUT_H);
		//Acc_z = read_raw_data(REG_ACCEL_ZOUT_H);
		
		Gyro_x = read_raw_data(REG_GYRO_XOUT_H);
		Gyro_y = read_raw_data(REG_GYRO_YOUT_H);
		//Gyro_z = read_raw_data(REG_GYRO_ZOUT_H);
		
      Tt = read_raw_data(REG_TEMP_OUT_H);
      TempC = (float)Tt/340 + 36.53;
      TempF = (TempC * 1.8) + 32.0;

		/* Divide raw value by sensitivity scale factor */
		Ax = Acc_x/16384.0;
		Ay = Acc_y/16384.0;
		//Az = Acc_z/16384.0;
		
		Gx = Gyro_x/131;
		Gy = Gyro_y/131;
		//Gz = Gyro_z/131;
		
      if (gGyroPass) {
         if (gAcc_x_h == 0.0)
            gAcc_x_h = Ax;
         else if (Ax > gAcc_x_h)
            gAcc_x_h = Ax;

         if (gAcc_y_h == 0.0)
            gAcc_y_h = Ay;
         else if (Ay > gAcc_y_h)
            gAcc_y_h = Ay;

         if (gGyro_x_h == 0.0)
            gGyro_x_h = Gx;
         else if (Gx > gGyro_x_h)
            gGyro_x_h = Gx;

         if (gGyro_y_h == 0.0)
            gGyro_y_h = Gy;
         else if (Gy > gGyro_y_h)
            gGyro_y_h = Gy;

         if (gAcc_x_l == 0.0)
            gAcc_x_l = Ax;
         else if (Ax < gAcc_x_l)
            gAcc_x_l = Ax;

         if (gAcc_y_l == 0.0)
            gAcc_y_l = Ay;
         else if (Ay < gAcc_y_l)
            gAcc_y_l = Ay;

         if (gGyro_x_l == 0.0)
            gGyro_x_l = Gx;
         else if (Gx < gGyro_x_l)
            gGyro_x_l = Gx;

         if (gGyro_y_l == 0.0)
            gGyro_y_l = Gy;
         else if (Gy < gGyro_y_l)
            gGyro_y_l = Gy;
      } else {
         gGyroPass = true;
      }
      if (agtDebug) {
         printf("Gx=%.3f °/s\tGy=%.3f °/s\tAx=%.3f g\tAy=%.3f g\tTemp=%.2f F\n", Gx, Gy, Ax, Ay, TempF);
         printf("HGx=%.3f °/s\tHGy=%.3f °/s\tHAx=%.3f g\tHAy=%.3f g\n", gGyro_x_h, gGyro_y_h, gAcc_x_h, gAcc_y_h);
         printf("LGx=%.3f °/s\tLGy=%.3f °/s\tLAx=%.3f g\tLAy=%.3f g\n", gGyro_x_l, gGyro_y_l, gAcc_x_l, gAcc_y_l);
      }
      return true;
   }
   return false;
}




