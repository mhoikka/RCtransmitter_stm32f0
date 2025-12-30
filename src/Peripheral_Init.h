/**
  ******************************************************************************
  * @file    Peripheral_Init.h
  * @author  Mitchell Hoikka
  * @version V1.2.0
  * @date    10-9-2024
  * @brief   Header for main.c module
  ******************************************************************************
  */

#include "stm32f0xx_nucleo.h"
#include "bme280_defs.h"

struct bme280_data get_sensor_reading();
void delay_microseconds(uint32_t usec, void *intf_ptr);
int8_t BME280_I2C_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *intf_ptr);
int8_t BME280_I2C_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *intf_ptr);
void I2C_transmit(uint8_t i2c_address, uint8_t reg_address, uint8_t i2c_data);
uint8_t * I2C_receive(uint8_t i2c_address, uint8_t reg_address, uint8_t * i2c_data, uint8_t numbytes);
uint8_t nrf24_read_register(uint8_t reg);
void nrf24_write_register(uint8_t reg, uint8_t value);
void nrf24_multiwrite_register(uint8_t reg, uint8_t *values, uint8_t num_bytes);
void nrf24_write_command(uint8_t COMMAND);
uint8_t * nrf24_multiread_register(uint8_t reg, uint8_t num_bytes, uint8_t *rxData);
void test_nrf24_connection();
void set_nrf24_SPI_CSN(uint8_t input);
void set_nrf24_SPI_CE(uint8_t input);
void MySPI_Init();
void HBridge_Peripherals_Init();
void Servo_Peripherals_Init();
void DriveServoControl(uint8_t angle);
void enableADC_batteryvoltagesense();
void ADCPeripherals_Init();
uint16_t * ADC_take_Readings();
float batteryVoltageMeasurement(uint16_t ADC_12bit);
void DriveACMotorVoltageController(uint32_t Period);
void NRF24L01p_Init();
void I2C_Settings_Init();
void UART_Settings_Init();
void ICM20948_init();
void getICM20948_ACCEL_GYRO_TEMPdata();
void transmit(void * data, uint8_t data_len, uint8_t data_size);