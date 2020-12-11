
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
// enum STATES {
//   DRIVING = 0xA,
//   RESTING = 0xB,
//   SEARCHING = 0xC
// };
// enum STATES STATE;
//==============================================================================

typedef enum {
  OFF,
  DRIVING
} robot_state_t;

robot_state_t state = OFF;

void check_status(int8_t code, const char *label) {
  //if (code < -1)
    printf("%s failed with %d\n", label, code);
}


nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
nrf_drv_spi_config_t spi_config = {
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

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

drv_pixy2_spi_t *pixy;
pid_loop_t rotateLoop, translateLoop;
int8_t focusIndex;
KobukiSensors_t sensors = {0};


void setup() {
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
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL));
  nrf_delay_ms(10);

  // We need to initialize the pixy object
  check_status(pixy_init(&pixy, &spi_instance), "initialize");
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


void loop() {
  switch(state) {
    case OFF: {
      if (is_button_pressed(&sensors))
        state = DRIVING;
      break;
    }	  
    case DRIVING: {
    if (!is_button_pressed(&sensors)) {
        state = OFF;
	break;
    }
	
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
      kobukiDriveDirect(-40, -40);

    printf("sig: %u area: %u age: %u offset: %ld numBlocks: %d\n", block->m_signature, block->m_width * block->m_height, block->m_age, panOffset, pixy->numBlocks);
#if 0 // for debugging
    printf("%ld %ld %ld %ld", rotateLoop.m_command, translateLoop.m_command, left, right);
#endif

  // no object detected, go into reset state
  } else {
    goto stop;
  }
  //return;

  break;
    }

  stop:
    //printf("here\n");
    pid_reset(&rotateLoop);
    pid_reset(&translateLoop);
    kobukiDriveDirect(0, 0);
    focusIndex = -1;
    break;
  }
}



int main(void) {
  //printf("here\n");
  setup();

  while (1) {
    // read sensors from robot
    //printf("here\n");
    kobukiSensorPoll(&sensors);
    //printf("%d\n", getVersion(pixy));
    //print_version(pixy->version);
    loop();
    //kobukiDriveDirect(-40, 40);
    
    //
  }
}
