/**
 * @file
 *
 * @brief IMU Sensor MPU6050 Driver Implementation
 *
 * @ingroup I2CSensorMPU6050
 */

#include "mpu6050_version.h"
#include "mpu6050_internal.h"

const char bus_path[] = "/dev/i2c-2";

/* for "strncpy()" */
#include <string.h>

/*************************************************************************
** Private Data Structures
*************************************************************************/
char MPU6050_Buffer[MPU6050_BUFFER_SIZE];

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                 */
/* Library Initialization Routine                                  */
/* cFE requires that a library have an initialization routine      */
/*                                                                 */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int32 MPU6050_Init(void)
{
    /*
     * Call a C library function, like strcpy(), and test its result.
     *
     * This is primary for a unit test example, to have more than
     * one code path to exercise.
     *
     * The specification for strncpy() indicates that it should return
     * the pointer to the destination buffer, so it should be impossible
     * for this to ever fail when linked with a compliant C library.
     */
    if (strncpy(MPU6050_Buffer, "SAMPLE DATA", sizeof(MPU6050_Buffer) - 1) != MPU6050_Buffer)
    {
        return CFE_STATUS_NOT_IMPLEMENTED;
    }

    /* ensure termination */
    MPU6050_Buffer[sizeof(MPU6050_Buffer) - 1] = 0;

    OS_printf("MPU6050 Lib Initialized.%s\n", MPU6050_VERSION_STRING);

    return CFE_SUCCESS;
}

/*
* INTERNAL FUNCTIONS
*/

int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff){
  int rv;
  uint8_t value[nr_bytes];
  i2c_msg msgs[] = {{
    .addr = i2c_address,
    .flags = 0,
    .buf = &data_address,
    .len = 1,
  }, {
    .addr = i2c_address,
    .flags = I2C_M_RD,
    .buf = value,
    .len = nr_bytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };
  uint16_t i;

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    printf("ioctl failed...\n");
  } else {

    free(*buff);
    *buff = malloc(nr_bytes * sizeof(uint8_t));

    for (i = 0; i < nr_bytes; ++i) {
      (*buff)[i] = value[i];
    }
  }

  return rv;
}

int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t val){
  uint8_t out[2] = { ptr, val };
  i2c_msg msgs[1] = {
    {
      .addr = dev->address,
      .flags = 0,
      .len = (uint16_t) sizeof(out),
      .buf = &out[0]
    }
  };

  return i2c_bus_transfer(dev->bus, &msgs[0], RTEMS_ARRAY_SIZE(msgs));
}

int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t **buff){

  int fd;
  int rv;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(1 * sizeof(uint8_t));

  uint16_t nr_bytes = (uint16_t) 1;
  uint16_t chip_address = (uint16_t) 0x68;
  uint8_t data_address = (uint8_t) register_add;

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = read_bytes(fd, chip_address, data_address, nr_bytes, &tmp);

  close(fd);

  (*buff)[0] = *tmp;
  free(tmp);

  return rv;
}

int sensor_mpu6050_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg){
  int err;

  uint16_t aux;
  uint8_t addr;
  uint8_t val;

  uint8_t *whoami;

  switch (command) {
    case SENSOR_MPU6050_BEGIN:
      // Sanity check
      whoami = NULL;
      sensor_mpu6050_get_reg_8(MPU6050_WHOAMI, &whoami);
      if ((*whoami) != 0x68) {
        err = -ENOTTY;
        break;
      }

      // Sensor configuration
      err =       sensor_mpu6050_set_reg_8(dev, PWR_MGT_1, 0x00);               //Temp sensor enabled, internal 8MHz oscillator and cycle disabled
      err = err + sensor_mpu6050_set_reg_8(dev, SIGNAL_PATH_RESET, 0x03); //Accelerometer, Gyroscope and Thermometer path reset
      err = err + sensor_mpu6050_set_reg_8(dev, ACCEL_CONFIG, 0x01);      //5Hz filter +-2g
      err = err + sensor_mpu6050_set_reg_8(dev, GYRO_CONFIG, 0x00);       //+-250 deg/s
      err = err + sensor_mpu6050_set_reg_8(dev, INT_ENABLE, 1<<WOM_EN);         //Disables interrupts in MPU6050
      break;

    case SENSOR_MPU6050_SET_REG:
      aux = *(uint16_t*) arg;
      addr  = (uint8_t)((aux & 0xFF00) >> 8);
      val   = (uint8_t)(aux & 0x00FF);
      err = sensor_mpu6050_set_reg_8(dev, addr, val);
      break;

    default:
      err = -ENOTTY;
      break;
  }

  return err;
}

int sensor_mpu6050_get_gyro_axis(uint8_t **buff, sensor_mpu6050_axis axis){

  int err = 0;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(2 * sizeof(uint8_t));

  switch (axis) {
    case X:
      err = sensor_mpu6050_get_reg_8(GYRO_XOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(GYRO_XOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Y:
      err = sensor_mpu6050_get_reg_8(GYRO_YOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(GYRO_YOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Z:
      err = sensor_mpu6050_get_reg_8(GYRO_ZOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(GYRO_ZOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    default:
      err = -1;
  }

  return err;
}

int sensor_mpu6050_get_accel_axis(uint8_t **buff, sensor_mpu6050_axis axis){

  int err = 0;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(2 * sizeof(uint8_t));

  switch (axis) {
    case X:
      err = sensor_mpu6050_get_reg_8(ACCEL_XOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(ACCEL_XOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Y:
      err = sensor_mpu6050_get_reg_8(ACCEL_YOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(ACCEL_YOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    case Z:
      err = sensor_mpu6050_get_reg_8(ACCEL_ZOUT_H, &tmp);
      (*buff)[0] = *tmp;
      tmp = NULL;
      err = err + sensor_mpu6050_get_reg_8(ACCEL_ZOUT_L, &tmp);
      (*buff)[1] = *tmp;
      break;

    default:
      err = -1;
  }

  return err;
}

/*
* PUBLIC FUNCTIONS
*/

int i2c_dev_register_sensor_mpu6050(const char *bus_path, const char *dev_path){
  i2c_dev *dev;
  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, MPU6050_ADDRESS);
  if (dev == NULL) {
    return -1;
  }

  dev->ioctl = sensor_mpu6050_ioctl;

  return i2c_dev_register(dev, dev_path);
}

int sensor_mpu6050_begin(int fd){

  return ioctl(fd, SENSOR_MPU6050_BEGIN, NULL);
}

int sensor_mpu6050_set_register(int fd, uint8_t reg, uint8_t val){
  uint16_t vector = (reg<<8)|(val);
  return ioctl(fd, SENSOR_MPU6050_SET_REG, &vector);
}

float sensor_mpu6050_get_accel(sensor_mpu6050_axis axis, float offset){
  int16_t var;

	uint8_t *tmp;
  tmp = NULL;

  int err = 0;

  tmp = NULL;
	err = sensor_mpu6050_get_accel_axis(&tmp, axis);
  var = (tmp[0]<<8)|(tmp[1]);

  if(err != 0)  // There was an error
    return -1;
  else
    return (float)(var * (9.81/16384.0)) - offset;
}

float sensor_mpu6050_get_gyro(sensor_mpu6050_axis axis, float offset){
  int16_t var;

	uint8_t *tmp;
  tmp = NULL;

  int err = 0;

  tmp = NULL;
	err = sensor_mpu6050_get_gyro_axis(&tmp, axis);
  var = (tmp[0]<<8)|(tmp[1]);

  if(err != 0)  // There was an error
    return -1;
  else
	 return (float)(var * (250.0/32768.0)) - offset;
}

float sensor_mpu6050_get_temp(void){
  int err = 0;

  uint8_t *tmp;
  tmp = NULL;

  uint8_t raw_data[2];

  err = sensor_mpu6050_get_reg_8(TEMP_OUT_H, &tmp);
  raw_data[0] = (*tmp);
  tmp = NULL;
  err = err + sensor_mpu6050_get_reg_8(TEMP_OUT_L, &tmp);
  raw_data[1] = (*tmp);

  free(tmp);

  if(err != 0){
    printf("There was an error when reading temperature registers...\n");
    return -1;
  }

  int16_t raw_temp = raw_data[0] << 8 | raw_data[1];

  return ((float)raw_temp / 340.0) + 36.53;
}
