
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
// Queue-like structure where each turn marks the beginning
// of a new element in the queue with its own distance, angle, etc.
typedef struct {
  float distance; 
  float angle;
  bool turn;
  bool linear;
} breadcrumb;
static breadcrumb bc_arr[100]; //Be able to retrace 100 steps
static uint32_t bc_counter = 0; // index in the queue
/*Encoders and distance variables.*/
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

  // conversion from encoder ticks to meters
  const float CONVERSION = 0.0006108;

  // calculate result here and return it
  // 65535
  uint16_t limit = 65535;
  // overflow case
  if (current_encoder < previous_encoder) {
    return ((float) previous_encoder - (float) current_encoder + limit) * CONVERSION;
  // non-overflow case
  } else {
    return ((float) current_encoder - (float) previous_encoder) * CONVERSION;
  }
}

//==============================================================================

char dist_trav_str[16];
float dist;
uint16_t drive_start_enc_right;
int main(void) {
  //printf("here\n");
  setup();
  bool in_scan = false;

  while (1) {
    kobukiSensorPoll(&sensors);

    /* FSM assumes no obstacles or distance limits.*/

    switch(STATE) {
      case AWAITING: {
        display_write("AWAITING", DISPLAY_LINE_0);
        kobukiDriveDirect(0, 0);
        in_scan = false;
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
          STATE = EXPLORE;
          in_scan = false;
          //backtracking
          bc_arr[bc_counter].angle = angle;
          bc_arr[bc_counter].turn = true;
          bc_arr[bc_counter].linear = false;
          bc_counter++;
          drive_start_enc_right = sensors.rightWheelEncoder;
          lsm9ds1_stop_gyro_integration();
        } else if (blocks > 0) {
          STATE = MOVE;
          bc_arr[bc_counter].angle = angle;
          bc_arr[bc_counter].turn = true;
          bc_arr[bc_counter].linear = false;
          bc_counter++;
          lsm9ds1_stop_gyro_integration();
        } else {
          STATE = AWAITING;
          lsm9ds1_stop_gyro_integration();
        }
        break;
      }
      /*Drives forward for 0.5m then goes back to scanning. Assumes a non-de-
      terministic direction to drive in, since scan will randomly position
      the direction of the ROMI.*/
      case EXPLORE: {
        // getting rid of display_write of "explore" for debug below
        //display_write("EXPLORE", DISPLAY_LINE_0);
        dist += measure_distance(sensors.rightWheelEncoder, drive_start_enc_right);
        drive_start_enc_right = sensors.rightWheelEncoder;
        // distance traveled to display for explore debug
        snprintf(dist_trav_str, 16, "dist: %f", dist);
        display_write(dist_trav_str, DISPLAY_LINE_0);
        //last_encoder = curr_encoder;
        kobukiDriveDirect(40, 40);
// interrupt with ble
        if (dist >= 0.5) {
          //STATE = SCAN;
          STATE = RETURN;
          bc_arr[bc_counter].distance = dist;
          bc_arr[bc_counter].turn = false;
          bc_arr[bc_counter].linear = true;
          bc_counter++;
          dist = 0.0;
          kobukiDriveDirect(0,0);
        }
        
// state = returning;
// bc_arrblahblkflafkglak
        break;
      }

      /*Drives towards object. Assumes no distance sensor for now.*/
      case MOVE: {
        display_write("MOVE", DISPLAY_LINE_0);
        // get active blocks from Pixy
        int8_t blocks = getBlocks(pixy, false, CCC_SIG_ALL, CCC_MAX_BLOCKS);
        block_t *block;
        focusIndex = acquireBlock(); // brought this over 2 to the left
        if (focusIndex != -1) { // If we've found a block, find it, track it
           block = trackBlock(focusIndex);

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
    } else {
        STATE = SCAN;
    }

        break;
      }

      case AVOID: {
        break;
      }

      case REACHED: {
        break;
      }

      case RETURN: {
        display_write("RETURN", DISPLAY_LINE_0);
        if (bc_counter < 0) {
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
        	snprintf(dist_trav_str, 16, "angle: %f", angle);
        	display_write(dist_trav_str, DISPLAY_LINE_0);
		char dist_trav_str_2[16];
        	snprintf(dist_trav_str_2, 16, "dist: %f", bc_arr[bc_counter].distance);
        	display_write(dist_trav_str_2, DISPLAY_LINE_1);
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
