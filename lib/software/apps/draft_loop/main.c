
// Safety First project
//
// Controls a Kobuki robot via input from Pixy2

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"

#include "pixy2_spi.h"
#include "pid.h"

#include "HCSR04.h"
#include "virtual_timer_linked_list.h"
#include "virtual_timer.h"
//==============================================================================
//State Encoding and Varaibles
//==============================================================================
typedef struct {
  float distance = 0.0;
  float angle = 0.0;
  bool turn = false;
  bool linear = false;
}breadcrumb;
static breadcrumb bc_arr[100]; //Be able to retrace 100 steps
static uint32_t bc_counter = 0;
/*Encoding for the BLE.*/
typedef enum {
  AWAITING = 0xA,
  SCAN = 0xB,
  EXPLORE = 0xC,
  MOVE = 0xD,
  AVOID = 0xE,
  REACHED = 0xF,
  RETURN = 0x0
} STATES ;
// set init state
STATES STATE = AWAITING;


nrf_drv_spi_t pixy_spi_instance = NRF_DRV_SPI_INSTANCE(1);
nrf_drv_spi_config_t pixy_spi_config = {
  .sck_pin = BUCKLER_SD_SCLK,
  .mosi_pin = BUCKLER_SD_MOSI,
  .miso_pin = BUCKLER_SD_MISO,
  .ss_pin = BUCKLER_SD_CS,
  .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
  .orc = 0,
  .frequency = NRF_DRV_SPI_FREQ_4M,
  .mode = NRF_DRV_SPI_MODE_3,
  .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
};

nrf_drv_spi_t lcd_spi_instance = NRF_DRV_SPI_INSTANCE(2);
nrf_drv_spi_config_t lcd_spi_config = {
  .sck_pin = BUCKLER_LCD_SCLK,
  .mosi_pin = BUCKLER_LCD_MOSI,
  .miso_pin = BUCKLER_LCD_MISO,
  .ss_pin = BUCKLER_LCD_CS,
  .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
  .orc = 0,
  .frequency = NRF_DRV_SPI_FREQ_4M,
  .mode = NRF_DRV_SPI_MODE_3,
  .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
};

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

drv_pixy2_spi_t *pixy;
pid_loop_t rotateLoop, translateLoop;
int8_t focusIndex;
KobukiSensors_t sensors = {0};

/*Encoders and distance variables.*/
static uint16_t last_encoder = 0;
static float distance_traveled = 0.0;
/*Control Signals.*/
bool avoid_backup = false;
bool avoid_move = false;
bool reached_turning = false;
bool reached_final = false;
bool reached_left = false;
bool reached_approach = false;
bool reached_center = false;
bool return_turning = false;
bool return_action_done = false;
bool return_linear_turn = false;
bool return_linear_turn_done = false;
/*PID variables
he process of tuning the PID parameters (Kp, Ki and Kd) is a continuous trial
and error process. There is no exact way to calculate the value for the
parameters unless the whole system is mathematically modeled and simulated.
*/
int32_t integral = 0;//Accumaltes error over time.
float Kp;//Can be used to set values;
float Ki;
//==============================================================================
//Functions
//==============================================================================
/*Prints Error Messages.*/
void check_status(int8_t code, const char *label) {
  if (code < 0)
    printf("%s failed with %d\n", label, code);
}
/*Initializes SPI and Pixy hardware.*/
void setup() {
  //BLE setup
  setup_ble(&STATE);
  // initialize RTT library
  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  APP_ERROR_CHECK(nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config));
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize spi master
  APP_ERROR_CHECK(nrf_drv_spi_init(&pixy_spi_instance, &pixy_spi_config, NULL, NULL));
  APP_ERROR_CHECK(nrf_drv_spi_init(&lcd_spi_instance, &lcd_spi_config, NULL, NULL));
  nrf_delay_ms(10);

  display_init(&lcd_spi_instance);

  // We need to initialize the pixy object
  check_status(pixy_init(&pixy, &pixy_spi_instance), "initialize");
  print_version(pixy->version);
  nrf_delay_ms(10);

  // Use color connected components program for the pan tilt to track
  check_status(changeProg(pixy, PIXY_PROG_COLOR_CODE), "change program");
  nrf_delay_ms(10);

  check_status(getResolution(pixy), "resolution");
  nrf_delay_ms(10);

  pid_init(&rotateLoop, 1, 0, 0, false);
  pid_init(&translateLoop, 1, 0, 0, false);
  printf("initalize complete..\n");
  kobukiInit();
}

// Take the biggest block (blocks[0]) that's been around for at least 30 frames (1/2 second)
// and return its index, otherwise return -1
int16_t acquireBlock() {
  if (pixy->numBlocks > 0)// && pixy->blocks[0].m_age > 50)
    return pixy->blocks[0].m_signature;

  return -1;
}

// Find the block with the given index.  In other words, find the same object in the current
// frame -- not the biggest object, but he object we've locked onto in acquireBlock()
// If it's not in the current frame, return NULL
block_t *trackBlock(int8_t index) {
  uint8_t i;

  for (i=0; i < pixy->numBlocks; i++) {
    if (pixy->blocks[i].m_signature == index)
      return &pixy->blocks[i];
  }

  return NULL;
}
/*Test loop for pixy.*/
void loop() {
  //printf("FPS %d\n", getFPS(pixy));

  // get active blocks from Pixy
  int8_t blocks = getBlocks(pixy, false, CCC_SIG_ALL, CCC_MAX_BLOCKS);
  //printf("here\n");

  check_status(blocks, "blocks");
  if (blocks <= 0)
    //printf("here\n");
    goto stop;

  block_t *block;
  if (focusIndex == -1) { // search....
    printf("Searching for block...\n");
    focusIndex = acquireBlock();
    if (focusIndex >= 0)
      printf("Found block!\n");
  }
  if (focusIndex != -1) // If we've found a block, find it, track it
     block = trackBlock(focusIndex);

  // If we're able to track it, move motors
  if (block != NULL) {
    // calculate pan and tilt "errors" with respect to first object (blocks[0]),
    // which is the biggest object (they are sorted by size).
    int32_t panOffset = (int32_t)pixy->frameWidth/2 - (int32_t)block->m_x;
    int32_t tiltOffset = (int32_t)block->m_y - (int32_t)pixy->frameHeight/2;

    // update loops
    pid_update(&rotateLoop, panOffset);
    pid_update(&translateLoop, -tiltOffset);

    // calculate left and right wheel velocities based on rotation and translation velocities
    int8_t left = -rotateLoop.m_command + translateLoop.m_command;
    int8_t right = rotateLoop.m_command + translateLoop.m_command;

    // set wheel speeds
    if (panOffset < -20)
      kobukiDriveDirect(40, -40);
    else if (panOffset > 20)
      kobukiDriveDirect(-40, 40);
    else
      kobukiDriveDirect(0, 0);

    printf("sig: %u area: %u age: %u offset: %ld numBlocks: %d\n",
    block->m_signature, block->m_width * block->m_height, block->m_age, panOffset, pixy->numBlocks);
#if 0 // for debugging
    printf("%ld %ld %ld %ld", rotateLoop.m_command, translateLoop.m_command, left, right);
#endif

  // no object detected, go into reset state
  } else {
    goto stop;
  }
  return;

  stop:
    //printf("here\n");
    pid_reset(&rotateLoop);
    pid_reset(&translateLoop);
    kobukiDriveDirect(0, 0);
    focusIndex = -1;
}

/* Measures distance from the encoder. */
static float measure_distance(uint16_t current_encoder,
                              uint16_t previous_encoder) {
  const float CONVERSION = 0.0006108;
  uint16_t ticks = current_encoder >= previous_encoder
    ? current_encoder - previous_encoder
    : current_encoder + (UINT16_MAX - previous_encoder);
  return CONVERSION * ticks;
}

//==============================================================================

int main(void) {
  //printf("here\n");
  setup();
  setup_dist();
  bool in_scan = false;

  while (1) {
    kobukiSensorPoll(&sensors);


    switch(STATE) {
      /*IDlE and RESET state,*/
      case AWAITING: {
        display_write("AWAITING", DISPLAY_LINE_0);
        kobukiDriveDirect(0, 0);
        in_scan = false;
        bc_counter = 0;
        break;
      }
      /*Rotates 360 looking for at least 1 block. If it finds at least blocks
      it moves into its general direction. Otherwise it goes to explore.
      Default case is to go back into AWAITING.*/
      case SCAN: {
        display_write("SCAN", DISPLAY_LINE_0);
        if(!in_scan) {
          in_scan = true;
          lsm9ds1_start_gyro_integration();
        }

        // get active blocks from Pixy
        float angle = fabs(lsm9ds1_read_gyro_integration().z_axis);
        int8_t blocks = getBlocks(pixy, false, CCC_SIG_ALL, CCC_MAX_BLOCKS);

        if (blocks <= 0 && angle < 360) {
          kobukiDriveDirect(-40, 40);
        } else if (blocks <= 0 && angle > 360) {
          bc_arr[bc_counter].turn = true;
          bc_arr[bc_counter].angle = angle;
          bc_counter++; 
          STATE = EXPLORE;
          lsm9ds1_stop_gyro_integration();
        } else if (blocks > 0) {
          bc_arr[bc_counter].turn = true;
          bc_arr[bc_counter].angle = angle;
          bc_counter++;
          STATE = MOVE;
          lsm9ds1_stop_gyro_integration();
        } else {
          STATE = AWAITING;
          lsm9ds1_stop_gyro_integration();
        }
        break;
      }
      /*Drives forward for 0.5m then goes back to scanning. Assumes a non-de-
      terministic direction to drive in, since scan will randomly position
      the direction of the ROMI.

      It will also check to see if there is an object at least 10cm away from it
      */
      case EXPLORE: {
        display_write("EXPLORE", DISPLAY_LINE_0);
        uint16_t curr_encoder = sensors.leftWheelEncoder;
        float value = measure_distance(curr_encoder, last_encoder);
        distance_traveled += value;
        last_encoder = curr_encoder;
        kobukiDriveDirect(-40, -40);
        float dist;
        get_distance(&dist);
        if (dist <= 10) {
          bc_arr[bc_counter].linear = true;
          bc_arr[bc_counter].distance = distance_traveled;
          bc_counter++;
          STATE = AVOID;
          avoid_move = false;
          break;
        } else if (distance_traveled >= 0.5) {
          bc_arr[bc_counter].linear = true;
          bc_arr[bc_counter].distance = distance_traveled;
          bc_counter++;
          STATE = SCAN;
          distance_traveled = 0.0;
          kobukiDriveDirect(0,0);
        }
        break;
      }

      /*Drives towards object. Assumes no distance sensor for now.*/
      case MOVE: {
        display_write("MOVE", DISPLAY_LINE_0);
        // get active blocks from Pixy
        int8_t blocks = getBlocks(pixy, false, CCC_SIG_ALL, CCC_MAX_BLOCKS);
        block_t *block;
        focusIndex = acquireBlock(); // brought this over 2 to the left
        if (focusIndex != -1) // If we've found a block, find it, track it
           block = trackBlock(focusIndex);
        float dist;
        //Detect if there is an object in front that is not the tracked block.
        //Or detect if we are close to the target.
        get_distance(&dist);
        if (dist <=10 && focusIndex == -1) {
          STATE = AVOID;
          avoid_move = true;
          break;
        } else if (dist <= 10 && focusIndex != -1) {
          STATE = REACHED;
          lsm9ds1_start_gyro_integration();
          reached_turning = true;
          break;
        }
        // If we're able to track it, move motors
        if (block != NULL) {
          // calculate pan and tilt "errors" with respect to first object (blocks[0]),
          // which is the biggest object (they are sorted by size).
          int32_t panOffset = (int32_t)pixy->frameWidth/2 - (int32_t)block->m_x;

          // adjust accordingly
          if (panOffset < -20)
            kobukiDriveDirect(-40, -50);
          else if (panOffset > 20)
            kobukiDriveDirect(-50, -40);
          else
            kobukiDriveDirect(-40, -40);

        // no object detected, go into reset state
        } else {
          focusIndex = -1;
        }
        break;
      }
      /*  2 Possible cases
      1) The target is obscured by the obstacle
      2) There is no target and only the obstacle

      Assuming only case 1 for now. The romi will back up 0.5m, and then stop
      and begin the gyro integration to turn either 45 degrees to the
      left/right.

      The next state will be determined by the control signal avoid_move. If
      true then the next state will be move, otherwise it will be explore.
      */
      case AVOID: {
        display_write("AVOID", DISPLAY_LINE_0);
        if (!avoid_backup) {
        uint16_t curr_encoder = sensors.leftWheelEncoder;
        float value = measure_distance(curr_encoder, last_encoder);
        distance_traveled += value;
        last_encoder = curr_encoder;
        kobukiDriveDirect(40, 40);
        if (distance_traveled >= 0.5) {
          lsm9ds1_start_gyro_integration();
          distance_traveled = 0.0;
          kobukiDriveDirect(0,0);
          avoid_backup = true;
        }
      } else {
          float angle = fabs(lsm9ds1_read_gyro_integration().z_axis);
          if (angle >= 45) {
            lsm9ds1_stop_gyro_integration();
            kobukiDriveDirect(0, 0);
            if (avoid_move) {
              STATE = MOVE;
              avoid_move = false;
            } else
              STATE = EXPLORE;
          } else
            kobukiDriveDirect(-40, 40);
        }
      break;
      }
      /*Romi will encircle the target by making a diamond shape followed by
      a 135 degree turn to be directly behind the target.

      Code here is longer than usual because REACHED requires it's own FSM.
      Instead of using a switch statement, here it's implemented
      with control signals.
      */
      case REACHED: {
        display_write("REACHED", DISPLAY_LINE_0);
        if(reached_turning && !reached_left) {
          float angle = fabs(lsm9ds1_read_gyro_integration().z_axis);
          if (angle >= 45) {
            reached_turning = false;
            distance_traveled = 0.0;
            kobukiDriveDirect(0,0);
            reached_left = true;
            lsm9ds1_stop_gyro_integration();
          } else {
            kobukiDriveDirect(-40, 40);
          }
        } else if (reached_turning && reached_left) {
          float angle = fabs(lsm9ds1_read_gyro_integration().z_axis);
          if (angle >= 45) {
            reached_turning = false;
            distance_traveled = 0.0;
            kobukiDriveDirect(0,0);
            reached_final = true;
            lsm9ds1_stop_gyro_integration();
            lsm9ds1_start_gyro_integration();
          } else {
            kobukiDriveDirect(40, -40);
          }
        }
        else if (!reached_turning && !reached_final){
          uint16_t curr_encoder = sensors.leftWheelEncoder;
          float value = measure_distance(curr_encoder, last_encoder);
          distance_traveled += value;
          last_encoder = curr_encoder;
          kobukiDriveDirect(-40, -40);
          if (distance_traveled >= 0.5) {
            reached_left = true;
            reached_turning = true;
            lsm9ds1_start_gyro_integration();
          }
        } else if (!reached_turning && reached_final)
          if (!reached_approach) {
            float angle = fabs(lsm9ds1_read_gyro_integration().z_axis);
            if (angle >= 130) {
              reached_approach = true;
              lsm9ds1_stop_gyro_integration();
              kobukiDriveDirect(0,0);
            } else {
              kobukiDriveDirect(40, -40);
            }
          } else {
            if (!reached_center) {
            //This is an experimental PID loop to center to the target.
            int8_t blocks = getBlocks(pixy, false, CCC_SIG_ALL, CCC_MAX_BLOCKS);
            block_t *block;
            focusIndex = acquireBlock(); // brought this over 2 to the left
            if (focusIndex != -1) // If we've found a block, find it, track it
               block = trackBlock(focusIndex);
           if (block != NULL) {
             // calculate pan and tilt "errors" with respect to first object (blocks[0]),
             // which is the biggest object (they are sorted by size).
             int32_t panOffset = (int32_t)pixy->frameWidth/2 - (int32_t)block->m_x;

             int32_t target = 0;
             int32_t error = target - panOffset;
             integral += error;
             //Play with integral and proportional gain, if needed.
             //These are guessed values.
             int32_t pwm = (5*error) + (integral*0.5);

             kobukiDriveDirect(pwm, pwm);

             if (error <= 10 && error >= -10) {
               reached_center = true;
               distance_traveled = 0.0;
             }

           } else {
             kobukiDriveDirect(-40, -40);
             float dist;
             get_distance(&dist);
             if (dist <= 10) {
               STATE = RETURN;
               break;
             }
             }
           }
           // no object detected, go into reset state
           } else {
             focusIndex = -1;
           }
          }
        break;

      /*Assumes the Romi is directly behind the Target to push it to the
      starting point.

      LIFO Algorithm unpacks the breadcrumb array and inverts each move to
      retrace its steps back to the original location.
      */
      case RETURN: {
        display_write("RETURN", DISPLAY_LINE_0);
        if (bc_counter == 0) {
          STATE = AWAITING;

        } else {
          if (bc_arr[bc_counter].turn) {
            if(!return_turning) {
              lsm9ds1_start_gyro_integration();
              return_turning = true;
            } else {
              float angle = fabs(lsm9ds1_read_gyro_integration().z_axis);
              if (bc_arr[bc_counter].angle < 0) {
                if (angle >= fabs(bc_arr[bc_counter].angle)) {
                  return_action_done = true;
                  distance_traveled = 0.0;
                  lsm9ds1_stop_gyro_integration();
                  break;
                } else
                    kobukiDriveDirect(40, -40);
              } else {
                if (angle >= fabs(bc_arr[bc_counter].angle)) {
                  return_action_done = true;
                  distance_traveled = 0.0;
                  lsm9ds1_stop_gyro_integration();
                  break;
                } else
                  kobukiDriveDirect(-40, 40);
              }
            }
          } else {
            if (!return_linear_turn_done) {
              if (!return_linear_turn) {
                lsm9ds1_start_gyro_integration();
                return_linear_turn = true;
              } else {
                float angle = fabs(lsm9ds1_read_gyro_integration().z_axis);
                if (angle >= 180) {
                  return_linear_turn_done = true;
                  lsm9ds1_stop_gyro_integration();
                  distance_traveled = 0.0;
                } else {
                  kobukiDriveDirect(-40, 40);
                }
              }
            } else {
            uint16_t curr_encoder = sensors.leftWheelEncoder;
            float value = measure_distance(curr_encoder, last_encoder);
            distance_traveled += value;
            last_encoder = curr_encoder;
            kobukiDriveDirect(-40, -40);
            if (distance_traveled >= bc_arr[bc_counter].distance) {
              return_action_done = true;
              return_linear_turn_done = false;
              return_linear_turn = false;
            }
          }
          }
        }
        if (return_action_done) {
          bc_counter--;
          return_action_done = false;
        }

        break;
      }
    }
  }
}
