// Display app
//
// Write messages to a Newhaven OLED display over SPI

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
// Pointers
/**
* @brief Function for starting the SPI data transfer.
*
* If an event handler was provided in the @ref nrf_drv_spi_init call, this function
* returns immediately and the handler is called when the transfer is done.
* Otherwise, the transfer is performed in blocking mode, which means that this function
* returns when the transfer is finished.
*
* @note Peripherals using EasyDMA (for example, SPIM) require the transfer buffers
*       to be placed in the Data RAM region. If they are not and an SPIM instance is
*       used, this function will fail with the error code NRF_ERROR_INVALID_ADDR.
*
* @param[in] p_instance       Pointer to the driver instance structure.
* @param[in] p_tx_buffer      Pointer to the transmit buffer. Can be NULL
*                             if there is nothing to send.
* @param     tx_buffer_length Length of the transmit buffer.
* @param[in] p_rx_buffer      Pointer to the receive buffer. Can be NULL
*                             if there is nothing to receive.
* @param     rx_buffer_length Length of the receive buffer.
*
* @retval NRF_SUCCESS            If the operation was successful.
* @retval NRF_ERROR_BUSY         If a previously started transfer has not finished
*                                yet.
* @retval NRF_ERROR_INVALID_ADDR If the provided buffers are not placed in the Data
*                                RAM region.
__STATIC_INLINE
ret_code_t nrf_drv_spi_transfer(nrf_drv_spi_t const * const p_instance,
                               uint8_t const * p_tx_buffer,
                               uint8_t         tx_buffer_length,
                               uint8_t       * p_rx_buffer,
                               uint8_t         rx_buffer_length);
*/

// You need to write send(), which takes a pointer to the data you want to send and the number of
// bytes to send via your serial port (SPI, I2C or UART).  It returns the number of bytes successfully sent.
ret_code_t send(uint8_t *data, uint8_t len, nrf_drv_spi_t const * const p_instance) {

  return nrf_drv_spi_transfer(p_instance, data, len, NULL, 0);
}

// You also need to write recv(), which takes a pointer to a data buffer where the received data
// will be written/returned and the number of bytes to receive via your serial port (SPI, I2C or UART).
// It returns the number of bytes immediately available and written into the buffer (without needing
// to busy-wait.)
ret_code_t recv(uint8_t *data, uint8_t len, nrf_drv_spi_t const * const p_instance) {

  return nrf_drv_spi_transfer(p_instance, NULL, 0, data, len);
}



int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("hello\n");

  // initialize spi master
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_SD_SCLK,
    .mosi_pin = BUCKLER_SD_MOSI,
    .miso_pin = BUCKLER_SD_MISO,
    .ss_pin = BUCKLER_SD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_2M,
    .mode = NRF_DRV_SPI_MODE_3,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);


uint8_t i, lenReceived, recvBuf[32];
uint8_t versionRequest[] =
{
  0xae,  // first byte of no_checksum_sync (little endian -> least-significant byte first)
  0xc1,  // second byte of no_checksum_sync
  0x0e,  // this is the version request type
  0x00   // data_length is 0
};



// clear out any stale data

//printf("%d\n", recv(recvBuf, 1, &spi_instance));

error_code = send(versionRequest, 4, &spi_instance);
printf("error code 2:%d\n", error_code);
nrf_delay_ms(1); // delay a little so we don't receive too fast (may not be necessary.)
error_code = recv(recvBuf, 6 + 16, &spi_instance); // 6 bytes of header and checksum and 16 bytes of version data
printf("error code 3:%d\n", error_code);
// print result
for (i=0; i<22; i++)
  printf("%hhu: 0x%hhx\n", i, recvBuf[i]);
}
