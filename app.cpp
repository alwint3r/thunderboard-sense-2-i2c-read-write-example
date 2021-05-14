/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
#include "em_gpio.h"
#include "em_i2c.h"
#include "sl_i2cspm_instances.h"
#include "ustimer.h"
#include "stdio.h"

#define SI1133_ADDR (0x55)

void app_init(void)
{
  // set PF9 as output and set the level to high to turn on the env sensor
  // this will enable Si1133 along with BMP280, Si7021
  GPIO_PinModeSet(gpioPortF, 9, gpioModePushPull, 1);

  // init timer
  USTIMER_Init();

  // wait for some time before the UV sensor is finished initializing
  // 25ms is the minimum start up time from datasheet
  USTIMER_DelayIntSafe(25000);
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  I2C_TransferSeq_TypeDef transfer;
  I2C_TransferReturn_TypeDef result;

  // address to read the PART_ID data from sensor
  uint8_t write_data = 0x00;

  // uint8_t is enough since we're only reading 1 byte.
  uint8_t rx_buf = 0;

  transfer.addr = (SI1133_ADDR << 1);
  transfer.buf[0].data = &write_data;
  transfer.buf[0].len = sizeof(write_data);
  transfer.buf[1].data = &rx_buf;
  transfer.buf[1].len = sizeof(rx_buf);

  // flags I2C_FLAG_WRITE_READ does:
  // write data from transfer.buf[0] to sensor
  // store read result to transfer.buf[1]
  transfer.flags = I2C_FLAG_WRITE_READ;

  result = I2CSPM_Transfer(sl_i2cspm_sensor_env, &transfer);

  printf("i2c transfer result: %d\r\n", (int) result);

  // should print 0x33
  printf("Part ID: 0x%x\r\n", rx_buf);

  USTIMER_Delay(3000 * 1000);
}
