#include <stdio.h> // for printf
#include <string.h> // for nstrcpy
#include <stdlib.h> // for free and malloc

#include "nrf_delay.h"

#include "pixy2_spi.h"
//#include "Pixy2Line.h"
//#include "Pixy2Video.h"


// Zed's debug macros

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

/*
 * I got some of the colour codes (as well as having the idea of putting them in a macro) from here:
 * http://stackoverflow.com/questions/3506504/c-code-changes-terminal-text-color-how-to-restore-defaults-linux
 */

#define RED    "\e[31m"
#define GREEN  "\e[32m"
#define YELLOW "\e[33m"
#define WHITE  "\e[1m"

/*
 * COLOR_X resets the colour.
 */
#define COLOR_X "\e[m"

#ifndef PIXY_DEBUG
  #define DEBUG(M, ...)
  #define TRACE()
#else
  #define DEBUG(M, ...) printf("DEBUG %s:%s:%d: " M "\n", __FILENAME__, __func__, __LINE__, ##__VA_ARGS__)
  //#define ASSERT(COND, ...) printf("DEBUG %s:%s:%d: assert " #(COND) " failed\n", __FILE__,  __func__, __LINE__, ##__VA_ARGS__)
  #define TRACE() printf("DEBUG %s:%s:%d\n", __FILENAME__, __func__, __LINE__)
#endif

#define LOG_ERR(M, ...) printf(RED "[ERROR]" COLOR_X " (%s:%s:%d) " M "\n", __FILENAME__, __func__, __LINE__  ##__VA_ARGS__)
#define LOG_WARN(M, ...) printf(YELLOW "[WARN]" COLOR_X " (%s:%s:%d) " M "\n", __FILENAME__, __func__, __LINE__, ##__VA_ARGS__)
#define LOG_INFO(M, ...) printf(WHITE "[INFO]" COLOR_X " (%s:%s:%d) " M "\n", __FILENAME__, __func__, __LINE__, ##__VA_ARGS__)


int8_t pixy_init(drv_pixy2_spi_t **pref, nrf_drv_spi_t const * const spi) {
  drv_pixy2_spi_t *p = malloc(sizeof(struct Pixy2));
  *pref = p;

  // allocate buffer space for send/receive
  p->m_buf = (uint8_t*)malloc(PIXY_BUFFERSIZE);
  // shifted buffer is used for sending, so we have space to write header information
  p->m_bufPayload = p->m_buf + PIXY_SEND_HEADER_SIZE;
  p->frameWidth = p->frameHeight = 0;
  p->version = NULL;

  p->spi = spi;

  p->blocks = NULL;
  p->numBlocks = 0;

  uint32_t t0;

  // wait for pixy to be ready -- that is, Pixy takes a second or 2 boot up
  // getVersion is an effective "ping".  We timeout after 5s.
  for(t0=0; t0<5; t0++) {
    if (getVersion(p) == PIXY_RESULT_OK) { // successful version get -> pixy is ready
      return getResolution(p); // get resolution so we have it
    }
  nrf_delay_ms(1000); // delay for sync
  }
  // timeout
  return PIXY_RESULT_TIMEOUT;
}


void pixy_close(drv_pixy2_spi_t *p) {
  free(p->m_buf);
}


int16_t getSync(drv_pixy2_spi_t *p) {
  TRACE();
  uint8_t i, j, c, cprev;
  int16_t res;
  uint16_t start;

  // parse bytes until we find sync
  for(i=j=0, cprev=0; true; i++) {
    res = recv(p->spi, &c, 1);
    if (res == NRF_SUCCESS) {
      // since we're using little endian, previous byte is least significant byte
      start = cprev;
      // current byte is most significant byte
      start |= c << 8;
      cprev = c;
      if (start==PIXY_CHECKSUM_SYNC) {
        p->m_cs = true;
        return PIXY_RESULT_OK;
      }
      if (start==PIXY_NO_CHECKSUM_SYNC) {
        p->m_cs = false;
        return PIXY_RESULT_OK;
      }
    }
    // If we've read some bytes and no sync, then wait and try again.
    // And do that several more times before we give up.
    // Pixy guarantees to respond within 100us.
    if (i>=4) {
      if (j>=4) {
        DEBUG("error: no response");
        return PIXY_RESULT_TIMEOUT;
      }
      nrf_delay_ms(25);
      j++;
      i = 0;
    }
  }
}


int16_t recvPacket(drv_pixy2_spi_t *p) {
  TRACE();
  uint16_t csCalc, csSerial;
  int16_t res;

  // clear out any stale data
  res = getSync(p);
  if (res == PIXY_RESULT_ERROR)
    return res;

  if (p->m_cs) {
    res = recv(p->spi, p->m_buf, 4);
    if (res != NRF_SUCCESS)
      return res;

    p->m_type = p->m_buf[0];
    p->m_length = p->m_buf[1];

    csSerial = *(uint16_t *)&p->m_buf[2];

    res = recv(p->spi, p->m_buf, p->m_length);
    if (res != NRF_SUCCESS)
      return res;

    csCalc = 0;
    for (uint8_t i=0; i<p->m_length; i++) {
      csCalc += p->m_buf[i];
    }

    if (csSerial!=csCalc) {
      DEBUG("error: checksum");
      return PIXY_RESULT_CHECKSUM_ERROR;
    }
  } else {
    res = recv(p->spi, p->m_buf, 2);
    if (res != NRF_SUCCESS)
      return res;

    p->m_type = p->m_buf[0];
    p->m_length = p->m_buf[1];

    res = recv(p->spi, p->m_buf, p->m_length);
    if (res != NRF_SUCCESS)
      return res;
  }
#ifdef PIXY_DEBUG
  DEBUG("received type=%d length=%d cs=%d", p->m_type, p->m_length, csSerial);
#endif
  return PIXY_RESULT_OK;
}


int16_t sendPacket(drv_pixy2_spi_t *p) {
  TRACE();
  // write header info at beginnig of buffer
  p->m_buf[0] = PIXY_NO_CHECKSUM_SYNC&0xff;
  p->m_buf[1] = PIXY_NO_CHECKSUM_SYNC>>8;
  p->m_buf[2] = p->m_type;
  p->m_buf[3] = p->m_length;
  // send whole thing -- header and data in one call
  send(p->spi, p->m_buf, p->m_length+PIXY_SEND_HEADER_SIZE);
  return PIXY_RESULT_OK;
}


int8_t changeProg(drv_pixy2_spi_t *p, const char *prog) {
  TRACE();
  int32_t res;

  // poll for program to change
  while(1) {
    strncpy((char *)p->m_bufPayload, prog, PIXY_MAX_PROGNAME);
    p->m_length = PIXY_MAX_PROGNAME;
    p->m_type = PIXY_TYPE_REQUEST_CHANGE_PROG;
    sendPacket(p);
    if ((res=recvPacket(p)) != PIXY_RESULT_OK)
      return res; // some kind of bitstream error
    res = *(uint32_t *)p->m_buf;
    if (res != 0) {
      return getResolution(p);  // get resolution so we have it
    }
    nrf_delay_ms(1000);
  }
}


int8_t getVersion(drv_pixy2_spi_t *p) {
  TRACE();
  int32_t res;

  p->m_length = 0;
  p->m_type = PIXY_TYPE_REQUEST_VERSION;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_VERSION) {
    p->version = (version_t *)p->m_buf;
    return p->m_length;
  } else if (p->m_type==PIXY_TYPE_RESPONSE_ERROR)
    return PIXY_RESULT_BUSY;
  else
    return PIXY_RESULT_ERROR;
}


int8_t getResolution(drv_pixy2_spi_t *p) {
  TRACE();
  int32_t res;

  p->m_length = 1;
  p->m_bufPayload[0] = 0; // for future types of queries
  p->m_type = PIXY_TYPE_REQUEST_RESOLUTION;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESOLUTION) {
    p->frameWidth = ((uint16_t*)p->m_buf)[0];
    p->frameHeight = ((uint16_t*)p->m_buf)[1];
    return PIXY_RESULT_OK; // success
  } else
    return PIXY_RESULT_ERROR;
}


int8_t setCameraBrightness(drv_pixy2_spi_t *p, uint8_t brightness) {
  uint32_t res;

  p->m_bufPayload[0] = brightness;
  p->m_length = 1;
  p->m_type = PIXY_TYPE_REQUEST_BRIGHTNESS;
  sendPacket(p);
  if ((res=recvPacket(p)) == PIXY_RESULT_OK) // && p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4)
  {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;
  } else
    return res;  // some kind of bitstream error
}


int8_t setServos(drv_pixy2_spi_t *p, uint16_t s0, uint16_t s1) {
  uint32_t res;

  *(int16_t *)(p->m_bufPayload + 0) = s0;
  *(int16_t *)(p->m_bufPayload + 2) = s1;
  p->m_length = 4;
  p->m_type = PIXY_TYPE_REQUEST_SERVO;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4) {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;
  } else
    return PIXY_RESULT_ERROR;
}


int8_t setLED(drv_pixy2_spi_t *p, uint8_t r, uint8_t g, uint8_t b) {
  uint32_t res;

  p->m_bufPayload[0] = r;
  p->m_bufPayload[1] = g;
  p->m_bufPayload[2] = b;
  p->m_length = 3;
  p->m_type = PIXY_TYPE_REQUEST_LED;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4) {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;
  } else
    return PIXY_RESULT_ERROR;
}

int8_t setLamp(drv_pixy2_spi_t *p, uint8_t upper, uint8_t lower) {
  uint32_t res;

  p->m_bufPayload[0] = upper;
  p->m_bufPayload[1] = lower;
  p->m_length = 2;
  p->m_type = PIXY_TYPE_REQUEST_LAMP;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4) {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;
  } else
      return PIXY_RESULT_ERROR;
}

int8_t getFPS(drv_pixy2_spi_t *p) {
  TRACE();
  uint32_t res;

  p->m_length = 0; // no args
  p->m_type = PIXY_TYPE_REQUEST_FPS;
  sendPacket(p);
  if ((res=recvPacket(p)) != PIXY_RESULT_OK)
    return res; // some kind of bitstream error
  if (p->m_type==PIXY_TYPE_RESPONSE_RESULT && p->m_length==4) {
    res = *(uint32_t *)p->m_buf;
    return (int8_t)res;
  } else
    return PIXY_RESULT_ERROR;  // some kind of bitstream error
}


// --- SPI ---


// You need to write send(), which takes a pointer to the data you want to send and the number of
// bytes to send via your serial port (SPI, I2C or UART).  It returns the number of bytes successfully sent.
void send(nrf_drv_spi_t const * const p_instance, uint8_t *data, uint8_t len) {
#ifdef PIXY_DEBUG
  printf("writing: ");
  for (int i=0; i<len; i++)
    printf("0x%hhx ", data[i]);
  printf("\n");
#endif
  APP_ERROR_CHECK(nrf_drv_spi_transfer(p_instance, data, len, NULL, 0));
}

// You also need to write recv(), which takes a pointer to a data buffer where the received data
// will be written/returned and the number of bytes to receive via your serial port (SPI, I2C or UART).
// It returns the number of bytes immediately available and written into the buffer (without needing
// to busy-wait.)
ret_code_t recv(nrf_drv_spi_t const * const p_instance, uint8_t *data, uint8_t len) {
  ret_code_t status = nrf_drv_spi_transfer(p_instance, NULL, 0, data, len);
#ifdef PIXY_DEBUG
  printf("reading: ");
  for (int i=0; i<len; i++)
    printf("0x%hhx ", data[i]);
  printf("\n");
  //printf("status: 0x%hhx\n", status);
#endif
  return status;
}


void print_version(version_t *v) {
    printf("hardware ver: 0x%x firmware ver: %d.%d.%d %s\n", v->hardware, v->firmwareMajor, v->firmwareMinor, v->firmwareBuild, v->firmwareType);
}


// print block structure!
void print_block(block_t *b) {
    int i, j;
    char sig[6], d;
    bool flag;
    if (b->m_signature > CCC_MAX_SIGNATURE) { // color code! (CC)
        // convert signature number to an octal string
        for (i=12, j=0, flag=false; i>=0; i-=3) {
            d = (b->m_signature >> i)&0x07;
            if (d>0 && !flag)
                flag = true;
            if (flag)
                sig[j++] = d + '0';
        }
        sig[j] = '\0';
        printf("CC block sig: %s (%d decimal) x: %d y: %d width: %d height: %d angle: %d index: %d age: %d\n", sig, b->m_signature, b->m_x, b->m_y, b->m_width, b->m_height, b->m_angle, b->m_index, b->m_age);
    } else // regular block.  Note, angle is always zero, so no need to print
        printf("sig: %d x: %d y: %d width: %d height: %d index: %d age: %d\n", b->m_signature, b->m_x, b->m_y, b->m_width, b->m_height, b->m_index, b->m_age);
}


int8_t getBlocks(drv_pixy2_spi_t *p, bool wait, uint8_t sigmap, uint8_t maxBlocks) {
  TRACE();

  while(1) {
    // fill in request data
    p->m_bufPayload[0] = sigmap;
    p->m_bufPayload[1] = maxBlocks;
    p->m_length = 2;
    p->m_type = CCC_REQUEST_BLOCKS;

    // send request
    sendPacket(p);
    if (recvPacket(p) == PIXY_RESULT_OK) {
      if (p->m_type==CCC_RESPONSE_BLOCKS) {
        p->blocks = (block_t*)p->m_buf;
        p->numBlocks = p->m_length/sizeof(struct Block);
        return p->numBlocks;
      }
    // deal with busy and program changing states from Pixy (we'll wait)
      else if (p->m_type == PIXY_TYPE_RESPONSE_ERROR) {
        if ((int8_t)p->m_buf[0] == PIXY_RESULT_BUSY) {
          if(!wait)
            return PIXY_RESULT_BUSY; // new data not available yet
    } else if ((int8_t)p->m_buf[0] != PIXY_RESULT_PROG_CHANGING)
          return p->m_buf[0];
      }
    } else
      return PIXY_RESULT_ERROR;  // some kind of bitstream error

    // If we're waiting for frame data, don't thrash Pixy with requests.
    // We can give up half a millisecond of latency (worst case)
    nrf_delay_ms(500);
  }
}
