/*************************************************************************
  Includes
*************************************************************************/
//#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include "SensorICM20948.h"

// InvenSense drivers and utils
#include "Icm20948.h"
#include "SensorTypes.h"
#include "Icm20948MPUFifoControl.h"
#include "Icm20948Setup.h"
#include "Icm20948SelfTest.h"

#include "../../i2c_bus.h"
#include "../imu.h"
#include <ch.h>
#include <hal.h>
#include "usbcfg.h"
#include "chprintf.h"

/*************************************************************************
  Variables
*************************************************************************/

int chipSelectPin;
int com_speed;

uint8_t I2C_Address = 0x68;

float gyro[3];
bool gyro_data_ready = false;
int32_t gyro_raw_data[3];
bool gyro_raw_data_ready = false;

float accel[3];
bool accel_data_ready = false;
int32_t accel_raw_data[3];
bool accel_raw_data_ready = false;

float mag[3];
bool mag_data_ready = false;
int32_t mag_raw_data[3];
bool mag_raw_data_ready = false;

float grav[3];
bool grav_data_ready = false;

float lAccel[3];
bool linearAccel_data_ready = false;

float quat6[4];
bool quat6_data_ready = false;

float euler6[3];
bool euler6_data_ready = false;

float quat9[4];
bool quat9_data_ready = false;

float euler9[3];
bool euler9_data_ready = false;

int har;
bool har_data_ready = false;

unsigned long steps;
bool steps_data_ready = false;

int fd_i2c = -1; // I2C file descriptor

// Required by abort()
void _exit(int status) {
    // In an embedded system, this usually means halting or resetting
    while(1) {
        // Halt the system
    }
}

// Required by _kill_r
int _kill(int pid, int sig) {
    // Return an error code (e.g., -1) to signify failure or unsupported operation
    return -1;
}

// Required by _getpid_r
int _getpid(void) {
    // Return a dummy process ID (e.g., 1)
    return 1;
}

/*************************************************************************
  HAL Functions
*************************************************************************/

int i2c_master_write_register(uint8_t reg, uint32_t len, const uint8_t *data)
{
	return write_reg_multi(I2C_Address, reg, data, len);
}

int i2c_master_read_register(uint8_t reg, uint32_t len, uint8_t *buff)
{
	return read_reg_multi(I2C_Address, reg, buff, len);
}


/*************************************************************************
  Invensense Variables
*************************************************************************/

inv_icm20948_t icm_device;
int rc = 0;
static const uint8_t EXPECTED_WHOAMI[] = { 0xEA }; /* WHOAMI value for ICM20948 or derivative */
#define AK0991x_DEFAULT_I2C_ADDR  0x0C
#define AK0991x_SECONDARY_I2C_ADDR  0x0E  /* The secondary I2C address for AK0991x Magnetometers */

#define THREE_AXES 3
static int unscaled_bias[THREE_AXES * 2];

static const float cfg_mounting_matrix[9] = {
  1.f, 0, 0,
  0, 1.f, 0,
  0, 0, 1.f
};

int32_t cfg_acc_fsr = 2; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 250; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

static const uint8_t dmp3_image[] = {
#include "icm20948_img.dmp3a.h"
};

/*************************************************************************
  Invensense Functions
*************************************************************************/

void check_rc(int rc, const char * msg_context)
{
  if (rc < 0) {
	  chprintf((BaseSequentialStream *)&SDU1, "ICM20948 ERROR! (%s)\n", msg_context);
    while (1);
  }
}

int load_dmp3(void)
{
  int rc = 0;
  rc = inv_icm20948_load(&icm_device, dmp3_image, sizeof(dmp3_image));
  return rc;
}

void inv_icm20948_sleep(int ms)
{
	chThdSleepMilliseconds(ms);
}

uint64_t inv_icm20948_get_time_us(void)
{
	// Not implemented yet since not strictly necessary.
	// ChibiOS has 1 ms resolution thus we would need a dedicated timer...
	return 0;
}

//---------------------------------------------------------------------
int idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
  return i2c_master_read_register(reg, rlen, rbuffer);
}

//---------------------------------------------------------------------

int idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
  return i2c_master_write_register(reg, wlen, wbuffer);
}

static void icm20948_apply_mounting_matrix(void)
{
  int ii;

  for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++) {
    inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, ii);
  }
}

static void icm20948_set_fsr(void)
{
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
  inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}

int icm20948_sensor_setup(void)
{

  int rc;
  uint8_t i, whoami = 0xff;

  inv_icm20948_soft_reset(&icm_device);

  // Get whoami number
  rc = inv_icm20948_get_whoami(&icm_device, &whoami);

  // Check if WHOAMI value corresponds to any value from EXPECTED_WHOAMI array
  for (i = 0; i < sizeof(EXPECTED_WHOAMI) / sizeof(EXPECTED_WHOAMI[0]); ++i) {

    if (whoami == EXPECTED_WHOAMI[i]) {
      break;
    }
  }

  if (i == sizeof(EXPECTED_WHOAMI) / sizeof(EXPECTED_WHOAMI[0])) {
	chprintf((BaseSequentialStream *)&SDU1, "Bad WHOAMI value = 0x%x\n", whoami);
    return rc;
  }

  // Setup accel and gyro mounting matrix and associated angle for current board
  inv_icm20948_init_matrix(&icm_device);

  // set default power mode
  rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
  if (rc != 0) {
	chprintf((BaseSequentialStream *)&SDU1, "Icm20948 Initialization failed.\n");
    return rc;
  }

  // Configure and initialize the ICM20948 for normal use

  // Initialize auxiliary sensors
  inv_icm20948_register_aux_compass( &icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
  rc = inv_icm20948_initialize_auxiliary(&icm_device);
  if (rc == -1) {
	chprintf((BaseSequentialStream *)&SDU1, "Compass not detected...\n");
  }

  icm20948_apply_mounting_matrix();

  icm20948_set_fsr();

  // re-initialize base state structure
  inv_icm20948_init_structure(&icm_device);

  return 0;
}

static uint8_t icm20948_get_grv_accuracy(void)
{
  uint8_t accel_accuracy;
  uint8_t gyro_accuracy;

  accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
  gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
  return (min(accel_accuracy, gyro_accuracy));
}

static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
  INV_SENSOR_TYPE_ACCELEROMETER,
  INV_SENSOR_TYPE_GYROSCOPE,
  INV_SENSOR_TYPE_RAW_ACCELEROMETER,
  INV_SENSOR_TYPE_RAW_GYROSCOPE,
  INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
  INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
  INV_SENSOR_TYPE_BAC,
  INV_SENSOR_TYPE_STEP_DETECTOR,
  INV_SENSOR_TYPE_STEP_COUNTER,
  INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
  INV_SENSOR_TYPE_ROTATION_VECTOR,
  INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
  INV_SENSOR_TYPE_MAGNETOMETER,
  INV_SENSOR_TYPE_SMD,
  INV_SENSOR_TYPE_PICK_UP_GESTURE,
  INV_SENSOR_TYPE_TILT_DETECTOR,
  INV_SENSOR_TYPE_GRAVITY,
  INV_SENSOR_TYPE_LINEAR_ACCELERATION,
  INV_SENSOR_TYPE_ORIENTATION,
  INV_SENSOR_TYPE_B2S
};

void build_sensor_event_data(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg)
{
  float raw_bias_data[6];
  inv_sensor_event_t event;
  (void)context;
  uint8_t sensor_id = convert_to_generic_ids[sensortype];

  memset((void *)&event, 0, sizeof(event));
  event.sensor = sensor_id;
  event.timestamp = timestamp;
  switch (sensor_id)
  {
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
      memcpy(raw_bias_data, data, sizeof(raw_bias_data));
      memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
      memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
      memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
      break;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
      memcpy(raw_bias_data, data, sizeof(raw_bias_data));
      memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
      memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
      memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
      mag_raw_data[0] = event.data.mag.vect[0];
      mag_raw_data[1] = event.data.mag.vect[1];
      mag_raw_data[2] = event.data.mag.vect[2];
      mag_raw_data_ready = true;
      break;
    case INV_SENSOR_TYPE_GYROSCOPE:
      memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
      memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));

      // WE WANT THIS
      gyro[0] = event.data.gyr.vect[0];
      gyro[1] = event.data.gyr.vect[1];
      gyro[2] = event.data.gyr.vect[2];
      gyro_data_ready = true;
      break;

    case INV_SENSOR_TYPE_GRAVITY:
      memcpy(event.data.grav.vect, data, sizeof(event.data.grav.vect));
      event.data.grav.accuracy_flag = inv_icm20948_get_accel_accuracy();

      grav[0] = event.data.grav.vect[0];
      grav[1] = event.data.grav.vect[1];
      grav[2] = event.data.grav.vect[2];
      grav_data_ready = true;
      //add gravity
      break;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
        memcpy(event.data.linAcc.vect, data, sizeof(event.data.linAcc.vect));
        memcpy(&(event.data.linAcc.accuracy_flag), arg, sizeof(event.data.linAcc.accuracy_flag));

        // WE WANT THIS
      lAccel[0] = event.data.linAcc.vect[0];
      lAccel[1] = event.data.linAcc.vect[1];
      lAccel[2] = event.data.linAcc.vect[2];
      linearAccel_data_ready = true;
      break;
        //add linear acceleration
    case INV_SENSOR_TYPE_ACCELEROMETER:
      memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
      memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));

      // WE WANT THIS
      accel[0] = event.data.acc.vect[0];
      accel[1] = event.data.acc.vect[1];
      accel[2] = event.data.acc.vect[2];
      accel_data_ready = true;
      break;

    case INV_SENSOR_TYPE_MAGNETOMETER:
      memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
      memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));

      // WE WANT THIS
      mag[0] = event.data.mag.vect[0];
      mag[1] = event.data.mag.vect[1];
      mag[2] = event.data.mag.vect[2];
      mag_data_ready = true;
      break;

    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
    case INV_SENSOR_TYPE_ROTATION_VECTOR:
      memcpy(&(event.data.quaternion9DOF.accuracy), arg, sizeof(event.data.quaternion9DOF.accuracy));
      memcpy(event.data.quaternion9DOF.quat, data, sizeof(event.data.quaternion9DOF.quat));
      quat9[0] = event.data.quaternion9DOF.quat[0];
      quat9[1] = event.data.quaternion9DOF.quat[1];
      quat9[2] = event.data.quaternion9DOF.quat[2];
      quat9[3] = event.data.quaternion9DOF.quat[3];
      quat9_data_ready = true;
      euler9_data_ready = true;
      break;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
      memcpy(event.data.quaternion6DOF.quat, data, sizeof(event.data.quaternion6DOF.quat));
      event.data.quaternion6DOF.accuracy_flag = icm20948_get_grv_accuracy();

      // WE WANT THIS
      quat6[0] = event.data.quaternion6DOF.quat[0];
      quat6[1] = event.data.quaternion6DOF.quat[1];
      quat6[2] = event.data.quaternion6DOF.quat[2];
      quat6[3] = event.data.quaternion6DOF.quat[3];
      quat6_data_ready = true;
      euler6_data_ready = true;
      break;

    case INV_SENSOR_TYPE_BAC:
      memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));

      har = event.data.bac.event;
      har_data_ready = true;
      break;
    case INV_SENSOR_TYPE_PICK_UP_GESTURE:
    case INV_SENSOR_TYPE_TILT_DETECTOR:
    case INV_SENSOR_TYPE_STEP_DETECTOR:
    case INV_SENSOR_TYPE_SMD:
      event.data.event = true;
      break;
    case INV_SENSOR_TYPE_B2S:
      event.data.event = true;
      memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
      break;
    case INV_SENSOR_TYPE_STEP_COUNTER:
      memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));

      steps = event.data.step.count;
      steps_data_ready = true;
      break;
    case INV_SENSOR_TYPE_ORIENTATION:
      //we just want to copy x,y,z from orientation data
      memcpy(&(event.data.orientation), data, 3 * sizeof(float));
      break;

    case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
    	memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
        accel_raw_data[0] = event.data.raw3d.vect[0];
        accel_raw_data[1] = event.data.raw3d.vect[1];
        accel_raw_data[2] = event.data.raw3d.vect[2];
        accel_raw_data_ready = true;
    	break;

    case INV_SENSOR_TYPE_RAW_GYROSCOPE:
      memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
      gyro_raw_data[0] = event.data.raw3d.vect[0];
      gyro_raw_data[1] = event.data.raw3d.vect[1];
      gyro_raw_data[2] = event.data.raw3d.vect[2];
      gyro_raw_data_ready = true;
      break;

    default:
      return;
  }
}

static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor)
{
  switch (sensor)
  {
    case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
      return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
    case INV_SENSOR_TYPE_RAW_GYROSCOPE:
      return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
    case INV_SENSOR_TYPE_ACCELEROMETER:
      return INV_ICM20948_SENSOR_ACCELEROMETER;
    case INV_SENSOR_TYPE_GYROSCOPE:
      return INV_ICM20948_SENSOR_GYROSCOPE;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
      return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
      return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
    case INV_SENSOR_TYPE_BAC:
      return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
    case INV_SENSOR_TYPE_STEP_DETECTOR:
      return INV_ICM20948_SENSOR_STEP_DETECTOR;
    case INV_SENSOR_TYPE_STEP_COUNTER:
      return INV_ICM20948_SENSOR_STEP_COUNTER;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
      return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_ROTATION_VECTOR:
      return INV_ICM20948_SENSOR_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
      return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_MAGNETOMETER:
      return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
    case INV_SENSOR_TYPE_SMD:
      return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
    case INV_SENSOR_TYPE_PICK_UP_GESTURE:
      return INV_ICM20948_SENSOR_FLIP_PICKUP;
    case INV_SENSOR_TYPE_TILT_DETECTOR:
      return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
    case INV_SENSOR_TYPE_GRAVITY:
      return INV_ICM20948_SENSOR_GRAVITY;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
      return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
    case INV_SENSOR_TYPE_ORIENTATION:
      return INV_ICM20948_SENSOR_ORIENTATION;
    case INV_SENSOR_TYPE_B2S:
      return INV_ICM20948_SENSOR_B2S;
    default:
      return INV_ICM20948_SENSOR_MAX;
  }
}

void SensorICM20948_init(SensorICM20948Settings settings)
{

	chprintf((BaseSequentialStream *)&SDU1, "Initializing ICM-20948...\n");

  // Initialize icm20948 serif structure
  struct inv_icm20948_serif icm20948_serif;
  icm20948_serif.context   = 0; // no need
  icm20948_serif.read_reg  = idd_io_hal_read_reg;
  icm20948_serif.write_reg = idd_io_hal_write_reg;
  icm20948_serif.max_read  = 1024 * 16; // maximum number of bytes allowed per serial read
  icm20948_serif.max_write = 1024 * 16; // maximum number of bytes allowed per serial write
  icm20948_serif.is_spi = false;

  // Reset icm20948 driver states
  inv_icm20948_reset_states(&icm_device, &icm20948_serif);
  inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

  // Setup the icm20948 device
  rc = icm20948_sensor_setup();

  if (icm_device.selftest_done && !icm_device.offset_done)
  {
    // If we've run self test and not already set the offset.
    inv_icm20948_set_offset(&icm_device, unscaled_bias);
    icm_device.offset_done = 1;
  }

  // Now that Icm20948 device is initialized, we can proceed with DMP image loading
  // This step is mandatory as DMP image is not stored in non volatile memory
  rc += load_dmp3();
  check_rc(rc, "Error sensor_setup/DMP loading.");

  // Set mode
  inv_icm20948_set_lowpower_or_highperformance(&icm_device, settings.mode);

  // Set frequency
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), 1000 / settings.gyroscope_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_RAW_GYROSCOPE), 1000 / settings.gyroscope_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), 1000 / settings.accelerometer_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_RAW_ACCELEROMETER), 1000 / settings.accelerometer_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), 1000 / settings.magnetometer_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), 1000 / settings.quaternion6_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ROTATION_VECTOR), 1000 / settings.quaternion9_frequency);
  //rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR), 1000 / settings.quaternion9_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GRAVITY), 1000 / settings.gravity_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_LINEAR_ACCELERATION), 1000 / settings.linearAcceleration_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_BAC), 1000 / settings.har_frequency);
  rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_STEP_COUNTER), 1000 / settings.steps_frequency);



  // Enable / disable
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), settings.enable_gyroscope);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_RAW_GYROSCOPE), settings.enable_gyroscope);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), settings.enable_accelerometer);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_RAW_ACCELEROMETER), settings.enable_accelerometer);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), settings.enable_magnetometer);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), settings.enable_quaternion6);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ROTATION_VECTOR), settings.enable_quaternion9);
  //rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR), settings.enable_quaternion9);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GRAVITY), settings.enable_gravity);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_LINEAR_ACCELERATION), settings.enable_linearAcceleration);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_BAC), settings.enable_har);
  rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_STEP_COUNTER), settings.enable_steps);
}



void SensorICM20948_task()
{
  inv_icm20948_poll_sensor(&icm_device, (void*)0, build_sensor_event_data);
}

bool SensorICM20948_gyroDataIsReady()
{
  return gyro_data_ready;
}

bool SensorICM20948_accelDataIsReady()
{
  return accel_data_ready;
}


bool SensorICM20948_magDataIsReady()
{
  return mag_data_ready;
}

bool SensorICM20948_linearAccelDataIsReady()
{
  return linearAccel_data_ready;
}

bool SensorICM20948_gravDataIsReady()
{
  return grav_data_ready;
}

bool SensorICM20948_quat6DataIsReady()
{
  return quat6_data_ready;
}

bool SensorICM20948_euler6DataIsReady()
{
  return euler6_data_ready;
}

bool SensorICM20948_quat9DataIsReady()
{
    return quat9_data_ready;
}

bool SensorICM20948_euler9DataIsReady()
{
    return euler9_data_ready;
}

bool SensorICM20948_harDataIsReady()
{
    return har_data_ready;
}

bool SensorICM20948_stepsDataIsReady()
{
    return steps_data_ready;
}

void SensorICM20948_readGyroData(float *x, float *y, float *z)
{
  *x = gyro[0];
  *y = gyro[1];
  *z = gyro[2];
  gyro_data_ready = false;
}

void SensorICM20948_readAccelData(float *x, float *y, float *z)
{
  *x = accel[0];
  *y = accel[1];
  *z = accel[2];
  accel_data_ready = false;
}

void SensorICM20948_readMagData(float *x, float *y, float *z)
{
  *x = mag[0];
  *y = mag[1];
  *z = mag[2];
  mag_data_ready = false;
}

void SensorICM20948_readLinearAccelData(float* x, float* y, float* z)
{
    *x = lAccel[0];
    *y = lAccel[1];
    *z = lAccel[2];
    linearAccel_data_ready = false;
}

void SensorICM20948_readGravData(float* x, float* y, float* z)
{
    *x = grav[0];
    *y = grav[1];
    *z = grav[2];
    grav_data_ready = false;
}

void SensorICM20948_readQuat6Data(float *w, float *x, float *y, float *z)
{
  *w = quat6[0];
  *x = quat6[1];
  *y = quat6[2];
  *z = quat6[3];
  quat6_data_ready = false;
}

void SensorICM20948_readEuler6Data(float *roll, float *pitch, float *yaw)
{
    *roll = (atan2f(quat6[0]*quat6[1] + quat6[2]*quat6[3], 0.5f - quat6[1]*quat6[1] - quat6[2]*quat6[2]))* 57.29578f;
    *pitch = (asinf(-2.0f * (quat6[1]*quat6[3] - quat6[0]*quat6[2])))* 57.29578f;
	*yaw = (atan2f(quat6[1]*quat6[2] + quat6[0]*quat6[3], 0.5f - quat6[2]*quat6[2] - quat6[3]*quat6[3]))* 57.29578f + 180.0f;
    euler6_data_ready = false;
}

void SensorICM20948_readQuat9Data(float* w, float* x, float* y, float* z)
{
    *w = quat9[0];
    *x = quat9[1];
    *y = quat9[2];
    *z = quat9[3];
    quat9_data_ready = false;
}

void SensorICM20948_readEuler9Data(float* roll, float* pitch, float* yaw)
{
    *roll = (atan2f(quat9[0] * quat9[1] + quat9[2] * quat9[3], 0.5f - quat9[1] * quat9[1] - quat9[2] * quat9[2])) * 57.29578f;
    *pitch = (asinf(-2.0f * (quat9[1] * quat9[3] - quat9[0] * quat9[2]))) * 57.29578f;
    *yaw = (atan2f(quat9[1] * quat9[2] + quat9[0] * quat9[3], 0.5f - quat9[2] * quat9[2] - quat9[3] * quat9[3])) * 57.29578f + 180.0f;
    euler9_data_ready = false;
}

void SensorICM20948_readHarData(char* activity)
{

    char temp = 'n';
    switch (har)
    {
        case 1:
            temp = 'd';
            break;
        case 2:
            temp = 'w';
            break;
        case 3:
            temp = 'r';
            break;
        case 4:
            temp = 'b';
            break;
        case 5:
            temp = 't';
            break;
        case 6:
            temp = 's';
            break;
    }
    *activity = temp;
    har_data_ready = false;
}

void SensorICM20948_readStepsData(unsigned long* step_count)
{
    *step_count = steps;
    steps_data_ready = false;
}

uint8_t SensorICM20948_readRvAccuracy(void)
{
  //return icm20948_get_grv_accuracy();
  return inv_icm20948_get_rv_accuracy();
  //return inv_icm20948_get_gmrv_accuracy();
}

uint8_t SensorICM20948_readMagAccuracy(void)
{
  return inv_icm20948_get_mag_accuracy();
}

uint8_t SensorICM20948_readAccAccuracy(void)
{
  return inv_icm20948_get_accel_accuracy();
}

uint8_t SensorICM20948_readGyroAccuracy(void)
{
  return inv_icm20948_get_gyro_accuracy();
}

uint8_t SensorICM20948_read(float *gyro_, float *acc_, float *temp, float *magnet, int16_t *gyro_raw, int16_t *acc_raw, int16_t *gyro_offset, int16_t *acc_offset, uint8_t *status, int16_t *roll, int16_t *pitch, int16_t *yaw, uint8_t* acc_accur, uint8_t *gyro_accur, uint8_t *mag_accur)
{
	static float r, p, y;
	inv_icm20948_poll_sensor(&icm_device, (void*)0, build_sensor_event_data);

    if(status) {
    	*status = 0;
    }
    if (acc_raw)
    {
    	if(accel_raw_data_ready)
    	{
    		accel_raw_data_ready = false;
			// Change the sign of all axes to have -1g when the robot is still on the plane and the axis points upwards and is perpendicular to the surface.
    		acc_raw[X_AXIS] = -accel_raw_data[0];
    		acc_raw[Y_AXIS] = -accel_raw_data[1];
    		acc_raw[Z_AXIS] = -accel_raw_data[2];
    	}
    }
    if (acc_)
    {
    	if(accel_data_ready)
    	{
    		accel_data_ready = false;
			acc_[X_AXIS] = accel[0]*STANDARD_GRAVITY;
			acc_[Y_AXIS] = accel[1]*STANDARD_GRAVITY;
			acc_[Z_AXIS] = accel[2]*STANDARD_GRAVITY;
    	}
    }
    if (temp) {
    	*temp = 0; // Not yet supported
    }
    if (gyro_raw)
    {
    	if(gyro_raw_data_ready)
    	{
    		gyro_raw_data_ready = false;
			gyro_raw[X_AXIS] = gyro_raw_data[0];
			gyro_raw[Y_AXIS] = gyro_raw_data[1];
			gyro_raw[Z_AXIS] = gyro_raw_data[2];
    	}
    }
    if (gyro_)
    {
    	if(gyro_data_ready)
    	{
    		gyro_data_ready = false;
			gyro_[X_AXIS] = DEG2RAD(gyro[0]);
			gyro_[Y_AXIS] = DEG2RAD(gyro[1]);
			gyro_[Z_AXIS] = DEG2RAD(gyro[2]);
    	}
    }

    if(magnet){
    	if(mag_data_ready)
    	{
    		mag_data_ready = false;
    		magnet[X_AXIS] = mag[0];
    		magnet[Y_AXIS] = mag[1];
    		magnet[Z_AXIS] = mag[2];
    	}
    }

    if(euler9_data_ready)
    {
    	euler9_data_ready = false;
    	SensorICM20948_readEuler9Data(&r, &p, &y);
    	*roll = (int16_t)r;
    	*pitch = (int16_t)p;
    	*yaw = (int16_t)y;
    	*acc_accur = inv_icm20948_get_accel_accuracy();
    	*gyro_accur = inv_icm20948_get_gyro_accuracy();
    	*mag_accur = inv_icm20948_get_mag_accuracy();
    }

    return MSG_OK;
}

