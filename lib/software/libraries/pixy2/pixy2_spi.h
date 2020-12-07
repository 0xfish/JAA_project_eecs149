// Main Pixy template class.  This class takes a link class and uses
// it to communicate with Pixy over I2C, SPI, UART or USB using the 
// Pixy packet protocol.

#ifndef _TPIXY2_H
#define _TPIXY2_H

#include "nrf_drv_spi.h"
#include <stdint.h> // for int types
#include <stdbool.h>


// uncomment to turn on debug prints to console
//#define PIXY_DEBUG

#define PIXY_BUFFERSIZE                      0x104
#define PIXY_CHECKSUM_SYNC                   0xc1af
#define PIXY_NO_CHECKSUM_SYNC                0xc1ae
#define PIXY_SEND_HEADER_SIZE                4
#define PIXY_MAX_PROGNAME                    33

#define PIXY_TYPE_REQUEST_CHANGE_PROG        0x02
#define PIXY_TYPE_REQUEST_RESOLUTION         0x0c
#define PIXY_TYPE_RESPONSE_RESOLUTION        0x0f
#define PIXY_TYPE_REQUEST_VERSION            0x0e
#define PIXY_TYPE_RESPONSE_VERSION           0x0f
#define PIXY_TYPE_RESPONSE_RESULT            0x00
#define PIXY_TYPE_RESPONSE_ERROR             0x03
#define PIXY_TYPE_REQUEST_BRIGHTNESS         0x10
#define PIXY_TYPE_REQUEST_SERVO              0x12
#define PIXY_TYPE_REQUEST_LED                0x14
#define PIXY_TYPE_REQUEST_LAMP               0x16
#define PIXY_TYPE_REQUEST_FPS                0x18

#define PIXY_RESULT_OK                       0
#define PIXY_RESULT_ERROR                    -1
#define PIXY_RESULT_BUSY                     -2
#define PIXY_RESULT_CHECKSUM_ERROR           -3
#define PIXY_RESULT_TIMEOUT                  -4
#define PIXY_RESULT_BUTTON_OVERRIDE          -5
#define PIXY_RESULT_PROG_CHANGING            -6

// RC-servo values
#define PIXY_RCS_MIN_POS                     0
#define PIXY_RCS_MAX_POS                     1000L
#define PIXY_RCS_CENTER_POS                  ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2)

#define PIXY_PROG_COLOR_CODE                 "color_connected_components"
#define PIXY_PROG_LINE_FOLLOW                "line_tracking"
#define PIXY_PROG_VIDEO                      "video"


typedef struct Version {
  uint16_t hardware;
  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  uint16_t firmwareBuild;
  char firmwareType[10];   
} version_t;

void print_version(version_t *v);


// --- SPI ---


int8_t open(uint32_t arg);
ret_code_t recv(nrf_drv_spi_t const * const spi, uint8_t *buf, uint8_t len);
void send(nrf_drv_spi_t const * const spi, uint8_t *buf, uint8_t len);


// --- CCC ---


#define CCC_MAX_SIGNATURE                   7

#define CCC_RESPONSE_BLOCKS                 0x20
#define CCC_REQUEST_BLOCKS                  0x20

#define CCC_MAX_BLOCKS                      (PIXY_BUFFERSIZE-PIXY_SEND_HEADER_SIZE)/sizeof(struct Block)

// Defines for sigmap:
// You can bitwise "or" these together to make a custom sigmap.
// For example if you're only interested in receiving blocks
// with signatures 1 and 5, you could use a sigmap of 
// PIXY_SIG1 | PIXY_SIG5
#define CCC_SIG1                     1 
#define CCC_SIG2                     2
#define CCC_SIG3                     4
#define CCC_SIG4                     8
#define CCC_SIG5                     16
#define CCC_SIG6                     32
#define CCC_SIG7                     64
#define CCC_COLOR_CODES              128

#define CCC_SIG_ALL                  0xff // all bits or'ed together


typedef struct Block {
  uint16_t m_signature;
  uint16_t m_x;
  uint16_t m_y;
  uint16_t m_width;
  uint16_t m_height;
  int16_t m_angle;
  uint8_t m_index;
  uint8_t m_age;
} block_t;

void print_block(block_t *b);


// --- interface ---


typedef struct Pixy2 {
  version_t *version;
  uint16_t frameWidth;
  uint16_t frameHeight;

  uint8_t *m_buf;
  uint8_t *m_bufPayload;
  uint8_t m_type;
  uint8_t m_length;
  bool m_cs;

  // Color connected components, color codes
  block_t *blocks;
  int8_t numBlocks;

  // Line following
  //Pixy2Line line;

  // Video
  //Pixy2Video video;

  nrf_drv_spi_t const * spi;
} drv_pixy2_spi_t;


int8_t pixy_init(drv_pixy2_spi_t** p, nrf_drv_spi_t const * const spi);
void pixy_close(drv_pixy2_spi_t* p);

int8_t getVersion(drv_pixy2_spi_t* p);
int8_t changeProg(drv_pixy2_spi_t* p, const char *prog);
int8_t setServos(drv_pixy2_spi_t* p, uint16_t s0, uint16_t s1);
int8_t setCameraBrightness(drv_pixy2_spi_t* p, uint8_t brightness);
int8_t setLED(drv_pixy2_spi_t* p, uint8_t r, uint8_t g, uint8_t b);
int8_t setLamp(drv_pixy2_spi_t* p, uint8_t upper, uint8_t lower);
int8_t getResolution(drv_pixy2_spi_t* p);
int8_t getFPS(drv_pixy2_spi_t* p);

int8_t getBlocks(drv_pixy2_spi_t *p, bool wait, uint8_t sigmap, uint8_t maxBlocks);


// --- serial helper ---


int16_t getSync(drv_pixy2_spi_t* p);
int16_t recvPacket(drv_pixy2_spi_t* p);
int16_t sendPacket(drv_pixy2_spi_t* p);


#endif
