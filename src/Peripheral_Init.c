#include <stdio.h>
#include "stm32f0xx_conf.h"
#include "stm32f0xx_i2c.h"
#include "bme280.h"
#include "bme280_defs.h"
#include "stm32f0xx_spi.h"
#include "Peripheral_Init.h"
#include <string.h>

typedef struct my_device_context {
    I2C_InitTypeDef *I2C_InitStruct; // Pointer to the I2C handle
    uint16_t i2c_address;     // I2C address of the device
} my_device_context;
struct my_device_context ctx = {};
struct bme280_dev bme280_initparam;
struct bme280_data bme280_datastruct;

uint8_t NUM_REGISTERS_BME280 = 4;

// NRF24L01+ REGISTERS
uint8_t CONFIG = 0x00;
uint8_t CONFIG_SETTINGS = 0x00;
uint8_t ENAA = 0x01;
uint8_t SETUP_AW = 0x03;
uint8_t RF_SETUP = 0x06;
uint8_t RX_ADDR_P0 = 0x0A;
uint8_t RX_PW_P0 = 0x11;
uint8_t TX_ADDR = 0x10;
uint8_t FEATURE = 0x1D;
uint8_t WRITE_COMMAND = 0x20;
uint8_t WRITE_PAYLOAD_COMMAND = 0xA0; 
uint8_t WRITE_PAYLOAD_NOACK = 0xB0;
uint8_t READ_PAYLOAD_COMMAND = 0x60;
uint8_t READ_COMMAND = 0x00;
uint8_t STATUS_REG = 0x07;
uint8_t FLUSH_TX = 0xE1;

uint8_t NO_ACK = 0;
uint8_t ACK = 1;

// ICM20948 REGISTERS
static uint8_t ICM_20948_I2C_ADDRESS = 0x69;
static uint8_t REG_BANK_SEL = 0x7F;
static uint8_t USER_BANK_0 = (0 << 4);
static uint8_t USER_BANK_1 = (1 << 4);
static uint8_t USER_BANK_2 = (2 << 4);
static uint8_t USER_BANK_3 = (3 << 4);

// REGISTER BANK 2
static uint8_t LP_CONFIG = 0x05;
static uint8_t PWR_MGMT_1 = 0x06;
static uint8_t PWR_MGMT_2 = 0x07;
// REGISTER BANK 2
static uint8_t TEMP_CONFIG = 0x53; 
static uint8_t GYRO_SMPLRT_DIV = 0x00;
static uint8_t GYRO_CONFIG_1 = 0x01;
static uint8_t GYRO_CONFIG_2 = 0x02;
static uint8_t ACCEL_SMPLRT_DIV = 0x10;
static uint8_t ACCEL_CONFIG = 0x14;
static uint8_t ACCEL_CONFIG_2 = 0x15;
static uint8_t GYRO_ACCEL_START_REG = 0x2D; // ACCEL_XOUT_H register

static uint8_t MAG_START_REG = 0x11;
/**
 * @brief  Delay function for BME280 drivers.
 * @param  usec: specifies the delay time length, in 1 microsecond.
 * @retval None
 */
void __attribute__((optimize("O0"))) delay_microseconds(uint32_t usec, void *intf_ptr){
  for(volatile uint32_t counter = 0; counter < usec; counter++){
    //do nothing NOP instructions
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    __NOP();__NOP();__NOP();__NOP();__NOP();
    // this is nearly perfect timing
  }
}

/**
 * @brief Reads from the I2C bus
 *  @param reg_addr: Address of the first register, where data is going to be read
 * @param reg_data: Pointer to data buffer to store the read data
 * @param cnt: Number of bytes of data to be read
 * @param intf_ptr: Pointer to the interface pointer
 * @retval 0 if successful, 1 if not
 */
int8_t BME280_I2C_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *intf_ptr){
	// Send the register address to the sensor
	// This is essentially a partial write operation
  uint32_t dev_addr = BME280_I2C_ADDR_SEC;
	I2C1->CR2 = (dev_addr << 1) | (I2C_CR2_START) | (1 << 16) | (I2C_CR2_AUTOEND);
	I2C1->TXDR = reg_addr; 

  // Wait for not busy
  while (!(I2C1->ISR & I2C_ISR_BUSY)){};

	while ((I2C1->ISR & I2C_ISR_BUSY)){};

	I2C1->CR2 = (dev_addr << 1) | (I2C_CR2_START) | (cnt << 16) | (I2C_CR2_RD_WRN) | (I2C_CR2_AUTOEND);

	// Transfer all the data
    for (int i = 0; i < cnt; i++) {
        // Wait for RXNE
        while (!(I2C1->ISR & I2C_ISR_RXNE));
        reg_data[i] = I2C1->RXDR;
    }


	// Wait for not busy
  while (!(I2C1->ISR & I2C_ISR_BUSY)){};
	while ((I2C1->ISR & I2C_ISR_BUSY)){};
	// Return the status of the read operation

	return 0; // BME OK
}

/**
 * @brief Writes to the I2C bus
 * @param reg_addr: Address of the first register, where data is going to be written
 * @param reg_data: Pointer to data buffer that stores the data to be written
 * @param cnt: Number of bytes of data to be written
 * @param intf_ptr: Pointer to the interface pointer
 * @retval 0 if successful, 1 if not
 */
int8_t BME280_I2C_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *intf_ptr){
  uint32_t dev_addr = BME280_I2C_ADDR_SEC;
	I2C1->CR2 = (dev_addr << 1) | (I2C_CR2_START) | (cnt*2 << 16) | (I2C_CR2_AUTOEND);
	// transfer cnt bytes of data one register data byte and register address byte pair at a time
	// register address is not auto-incremented
	if (cnt <= NUM_REGISTERS_BME280){
		for(int i = 0; i < cnt; i++){
			// send the register address as a the control byte and the register data as a data byte
			I2C1->TXDR = reg_addr;
			while(!(I2C1->ISR & I2C_ISR_TXE));
			I2C1->TXDR = reg_data[i];
			// increment register address manually
			++reg_addr;
			while(!(I2C1->ISR & I2C_ISR_TXE));
		}
	// Wait for not busy
  while (!(I2C1->ISR & I2C_ISR_BUSY)){};
	while ((I2C1->ISR & I2C_ISR_BUSY)){};
	return 0;
	}
	else{
		return 1;
		//TODO Create an error function here? Maybe?
	}
}

/**
 * @brief enable the GPIO pins to control the H-bridge for the AC 3-phase motor voltage
 * @retval None
 */
void HBridge_Peripherals_Init(){
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE); // TODO collect all clock enables at start of program

  // Enable pins to control NMOS H-Bridge
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // High speed
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitTypeDef GPIO_InitStruct2;
  GPIO_InitStruct2.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStruct2.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct2.GPIO_Speed = GPIO_Speed_50MHz; // High speed
  GPIO_InitStruct2.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct2.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStruct2);

  GPIO_InitTypeDef GPIO_InitStruct3;
  GPIO_InitStruct3.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStruct3.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct3.GPIO_Speed = GPIO_Speed_50MHz; // High speed
  GPIO_InitStruct3.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct3.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOD, &GPIO_InitStruct3);
}

/**
 * @brief Initializes all of the peripherals necessary to control the servo motors using the STM32 chip
 * @retval None
 */
void Servo_Peripherals_Init(){

  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE); // enable clocks for GPIO peripheral
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  // Enable pins to control servos
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // High speed
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // High speed
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOF, &GPIO_InitStruct);

  // Enable pin to control test diode and remaining servos
  GPIO_InitTypeDef GPIO_InitStruct2;
  GPIO_InitStruct2.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_11|GPIO_Pin_10|GPIO_Pin_2; // Diode pin and L-R elevator control pins
  GPIO_InitStruct2.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct2.GPIO_Speed = GPIO_Speed_50MHz; // High speed
  GPIO_InitStruct2.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct2.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStruct2);
}

/**
 * @brief Tests the servo motor, TODO add support for changing angle of servo arm as well
 * @param angle In range 0-4 inclusive and specifies the angle of the servo motor arm by the equation 45 * angle, with units of degrees
 * Note that two of the servo motors used are different and don't need to exceed the specified PWM time spans to fire to the specified angle, as the SG90 do
 */
void DriveServoControl(uint8_t angle){ // For testing purposes only, moves all servos to same angle and resets them to 0 degrees after 2 seconds
  GPIO_SetBits(GPIOB, GPIO_Pin_12); // Turn on test diode to confirm setup function worked
  uint32_t time_on = 0;
  for(int i = 0; i < 100; i++){ // TODO test how fast loaded arm moves, unloaded arm is rated to move at 0.09 s + 0.01 s/60 degrees at 4.8 V so I gave it 150 ms here
    GPIO_SetBits(GPIOC, GPIO_Pin_13); // LFlap // Servo uses PWM to control angle of arm
    GPIO_SetBits(GPIOF, GPIO_Pin_7); // Rflap
    GPIO_SetBits(GPIOB, GPIO_Pin_2); // Lelevator
    GPIO_SetBits(GPIOB, GPIO_Pin_11); // Relevator
    GPIO_SetBits(GPIOB, GPIO_Pin_10); // Rudder
    
    time_on = 800 + angle * 500; // in microseconds
    delay_microseconds(time_on, NULL); // exceeds 1000 or 1500 ms (45-90 degree angle for servo arm) as specified to ensure motor operates
    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // LFlap 
    GPIO_ResetBits(GPIOF, GPIO_Pin_7); // Rflap
    GPIO_ResetBits(GPIOB, GPIO_Pin_2); // Lelevator
    GPIO_ResetBits(GPIOB, GPIO_Pin_11); // Relevator
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // Rudder
    delay_microseconds(20000 - time_on, NULL);
  }  
  for(int i = 0; i < 100; i++){ // TODO test how fast loaded arm moves, unloaded arm is rated to move at 0.09 s + 0.01 s/60 degrees at 4.8 V so I gave it 150 ms here
    GPIO_SetBits(GPIOC, GPIO_Pin_13); // LFlap // Servo uses PWM to control angle of arm
    GPIO_SetBits(GPIOF, GPIO_Pin_7); // Rflap
    GPIO_SetBits(GPIOB, GPIO_Pin_2); // Lelevator
    GPIO_SetBits(GPIOB, GPIO_Pin_11); // Relevator
    GPIO_SetBits(GPIOB, GPIO_Pin_10); // Rudder
    
    time_on = 800 + 0 * 500; // in microseconds
    delay_microseconds(time_on, NULL); // exceeds 1000 or 1500 ms (45-90 degree angle for servo arm) as specified to ensure motor operates
    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // LFlap 
    GPIO_ResetBits(GPIOF, GPIO_Pin_7); // Rflap
    GPIO_ResetBits(GPIOB, GPIO_Pin_2); // Lelevator
    GPIO_ResetBits(GPIOB, GPIO_Pin_11); // Relevator
    GPIO_ResetBits(GPIOB, GPIO_Pin_10); // Rudder
    delay_microseconds(20000 - time_on, NULL);
  } 
}

/**
 * @brief Enables the ADC for the battery voltage measurement pin
 * @param None
 * @retval None
 */
void enableADC_batteryvoltagesense(){
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // enable clocks for GPIO peripheral 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // enable clocks for ADC peripheral
  // Configure PC0 as analog input for battery voltage measurement
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; // TODO change this pin
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // TODO check these settings
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
 * @brief Initializes the ADC1 peripheral for battery voltage measurement
 * @retval None
 */
void ADCPeripherals_Init(){
  // Set up ADC10, ADC11, and ADC12 for joystick voltage measurement
  ADC_InitTypeDef ADC_InitStruct;
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStruct.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStruct);
}

/**
 * @brief Takes a single ADC reading from ADC10, ADC11, and ADC12
 * @retval 3 12 bit ADC reading in unsigned integer format, in a 16 bit integer package padded with 0s on the MSB side
 */
uint16_t * ADC_take_Readings(uint16_t * adc_readings){
  // Take ADC readings from ADC10, ADC11, and ADC12 // TODO make it possible to select which ADC to read from

  // make sure ADC peripheral clock was enabled earlier:
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // safe to call if already enabled

  // Enable ADC and wait until ADEN is set
  ADC_Cmd(ADC1, ENABLE);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET) {
    /* wait for ADC ready */
  }

  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE); // enable clocks for GPIO peripheral 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // enable clocks for ADC peripheral
  // Configure PC0 as analog input for joystick voltage measurement
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2; 
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // TODO check these settings
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  ADC_ChannelConfig(ADC1, ADC_Channel_10, ADC_SampleTime_239_5Cycles);
  ADC_StartOfConversion(ADC1);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  adc_readings[0] = ADC_GetConversionValue(ADC1);
  
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  delay_microseconds(32, NULL);

  ADC_ChannelConfig(ADC1, ADC_Channel_11, ADC_SampleTime_239_5Cycles);
  // Perform a dummy conversion after switching channel so the sample-and-hold can settle,
  // otherwise the first conversion may still reflect the previous channel.
  ADC_StartOfConversion(ADC1);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  (void)ADC_GetConversionValue(ADC1); // discard dummy result

  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  delay_microseconds(32, NULL);

  ADC_StartOfConversion(ADC1);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  adc_readings[1] = ADC_GetConversionValue(ADC1);
  
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  delay_microseconds(32, NULL);

  ADC_ChannelConfig(ADC1, ADC_Channel_12, ADC_SampleTime_239_5Cycles);
  // dummy conversion after channel switch for stability
  ADC_StartOfConversion(ADC1);
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  (void)ADC_GetConversionValue(ADC1); // discard dummy result

  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  delay_microseconds(32, NULL);

  ADC_StartOfConversion(ADC1);
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  adc_readings[2] = ADC_GetConversionValue(ADC1);
  return adc_readings;
}

/**
 * @brief Converts ADC measurement to voltage value of battery taking the voltage divider into account
 * @param ADC_12bit 12 bit measurement from ADC in unsigned integer format, in a 16 bit integer package padded with 0s on the MSB side
 * @retval Battery voltage
 */
float batteryVoltageMeasurement(uint16_t ADC_12bit){
  float Rtop = 300000;
  float Rbottom = 100000;
  float battery_voltage;
  battery_voltage = ADC_12bit * 3.3 * (Rbottom + Rtop) / (Rbottom * 0xFFF); // TODO document this equation
  return battery_voltage;
}

/**
 * @brief Repeatedly cycle the control signals for the H-bridge to control the AC voltage for the electric motor (square waves, three phase AC)
 * @param Period The period of the on-off control signal cycle in microseconds
 * @retval None
 */
void DriveACMotorVoltageController(uint32_t Period){

  uint32_t halfPeriod = Period/2;
  uint32_t thirdPeriod = Period/3;
  uint32_t sixthPeriod = Period/6;
  
  while(TRUE){ // TODO test if there should be a time delay between S1 turning off and S2 turning on and vice versa (there should be a GPIO pin voltage rise and fall time of 125 ns max)
    GPIO_SetBits(GPIOB, GPIO_Pin_6); // S1 on

    delay_microseconds(sixthPeriod, NULL); // 1/6 period elapsed 
    GPIO_ResetBits(GPIOD, GPIO_Pin_2); // S5 off
    GPIO_SetBits(GPIOC, GPIO_Pin_11); // S6 on

    delay_microseconds(thirdPeriod-sixthPeriod, NULL); // 1/3 period elapsed (delay timings are computed like this to avoid rounding errors from integer division truncations)
    GPIO_ResetBits(GPIOB, GPIO_Pin_3); // S4 off
    GPIO_SetBits(GPIOB, GPIO_Pin_4); // S3 on

    delay_microseconds(halfPeriod - thirdPeriod, NULL); // 1/2 period elapsed
    GPIO_ResetBits(GPIOB, GPIO_Pin_6); // S1 off
    GPIO_SetBits(GPIOB, GPIO_Pin_5); // S2 on

    delay_microseconds(2*thirdPeriod - halfPeriod, NULL); // 2/3 period elapsed
    GPIO_ResetBits(GPIOC, GPIO_Pin_11); // S6 off
    GPIO_SetBits(GPIOD, GPIO_Pin_2); // S5 on

    delay_microseconds(sixthPeriod, NULL); // 5/6 period elapsed
    GPIO_ResetBits(GPIOB, GPIO_Pin_4); // S3 off
    GPIO_SetBits(GPIOB, GPIO_Pin_3); // S4 on
    
    delay_microseconds(Period-(sixthPeriod+2*thirdPeriod), NULL); // Full period elapsed
    GPIO_ResetBits(GPIOB, GPIO_Pin_5); // S2 off
  }
}

/**
 * @brief  Initializes the I2C1 communication
 * @retval None
 */
void I2C_Settings_Init(){

  // Enable peripheral clock
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  // Use pins PB8 and PB9 for I2C (STM32F030R8T6)
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
	
  // I2C configuration
  I2C_InitTypeDef I2C_InitStruct;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
  I2C_InitStruct.I2C_DigitalFilter = 0x00;
  I2C_InitStruct.I2C_Timing = 0x00901D23; // TODO recheck these values if I2C issues occur
  I2C_Init(I2C1, &I2C_InitStruct);
  I2C_Cmd(I2C1, ENABLE);

  ctx.I2C_InitStruct = &I2C_InitStruct;
  ctx.i2c_address = 0x68; // TODO replace this address with the bme280 address or remove if unnecessary
}

/**
 * @brief Transmits a single byte over I2C
 * @param i2c_address 7 bit I2C address of the slave device
 * @param i2c_data byte of data to be transmitted
 * @retval None
 */
void I2C_transmit(uint8_t i2c_address, uint8_t reg_address, uint8_t i2c_data){
  // Wait until bus is idle
  while (I2C1->ISR & I2C_ISR_BUSY);

  // Clear STOP flag from previous transfer
  I2C1->ICR = I2C_ICR_STOPCF;

  // Configure transfer (WRITE, 1 byte)
  I2C1->CR2 =
      (i2c_address << 1) |           // Slave address
      (2 << 16)|
      I2C_CR2_START;                 // Generate START


  // Wait for TXIS or NACK
  while (!(I2C1->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));

  if (I2C1->ISR & I2C_ISR_NACKF) {
      I2C1->ICR = I2C_ICR_NACKCF;
      return;
  }

  // Send data
  I2C1->TXDR = reg_address;
  // Send data byte
  while(!(I2C1->ISR & I2C_ISR_TXIS)); // Wait again
  I2C1->TXDR = i2c_data;

  // Wait for transfer complete
  while (!(I2C1->ISR & (I2C_ISR_TC | I2C_ISR_NACKF))); // TODO Should check for NACK here too? Not sure.

  if (I2C1->ISR & I2C_ISR_NACKF) {
      I2C1->ICR = I2C_ICR_NACKCF;
      return;
  }
  
  // Generate STOP
  I2C1->CR2 |= I2C_CR2_STOP;

  // Wait for STOP to finish
  while (!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR = I2C_ICR_STOPCF;
}

/**
 * @brief Receives a single byte over I2C
 * @param uint8_t  i2c_address 7 bit I2C address of the slave device
 * @param uint8_t  reg_address register address to read from
 * @param uint8_t* i2c_data pointer to array to store received data
 * @param uint8_t  numbytes number of bytes to read
 * @retval Received byte of data
 */
uint8_t * I2C_receive(uint8_t i2c_address, uint8_t reg_address, uint8_t * i2c_data, uint8_t numbytes){

  // Populate array with error data
  for(int i = 0; i < numbytes; i++){
    i2c_data[i] = 0xFF;
  }

  // Wait until bus is idle
  while (I2C1->ISR & I2C_ISR_BUSY);

  // Clear STOP flag from previous transfer
  I2C1->ICR = I2C_ICR_STOPCF;

  // Configure transfer (WRITE, 1 byte)
  I2C1->CR2 =
      (i2c_address << 1) |           // Slave address
      (1 << 16)| 
      I2C_CR2_START;                 // Generate START


  // Wait for TXIS or NACK
  while (!(I2C1->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF)));

  if (I2C1->ISR & I2C_ISR_NACKF) {
      I2C1->ICR = I2C_ICR_NACKCF;
      return i2c_data;
  }

  // Send data
  I2C1->TXDR = reg_address;

  // Wait for transfer complete
  while (!(I2C1->ISR & (I2C_ISR_TC | I2C_ISR_NACKF))); // TODO Should check for NACK here too? Not sure. Do I need to check for TC when reading right afterwards?

  if (I2C1->ISR & I2C_ISR_NACKF) {
      I2C1->ICR = I2C_ICR_NACKCF;
      return i2c_data;
  }
  
  // Configure transfer for reading
  I2C1->CR2 =
      (i2c_address << 1) |           // Slave address
      (numbytes << 16)|
      I2C_CR2_RD_WRN |              // Read mode
      I2C_CR2_START  |              // Generate START
	    I2C_CR2_AUTOEND;
  
  // Wait for RXNE or NACK 'numbytes' times and fill the data array, returning early if NACK received
  for(int i = 0; i < numbytes; i++){
    while (!(I2C1->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF)));
    if (I2C1->ISR & I2C_ISR_NACKF) {
      I2C1->ICR = I2C_ICR_NACKCF;
      // Repopulate array with error data
      for(int i = 0; i < numbytes; i++){
        i2c_data[i] = 0xFF;
      }
      return i2c_data; 
    }
    // Read data received
    i2c_data[i] = I2C1 -> RXDR;
  }
  
  // Wait for STOP to finish
  while (!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR = I2C_ICR_STOPCF;
  
  return i2c_data;
}

void nrf24_write_register(uint8_t reg, uint8_t value) {
    uint8_t txData[2]; // Transmit data buffer

    // Prepare command to write (register address with write command prefix)
    txData[0] = reg | WRITE_COMMAND; // Write command
    txData[1] = value;      // Data to write

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    
    // Start SPI transmission
    for (int i = 0; i < 2; i++) {
      // Transmit byte
      *(__IO uint8_t*)(&SPI1->DR) = txData[i]; 

      // Wait until transmission is complete
      while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

      // Read received byte (not used, but necessary to complete the transaction) 
      (void)SPI1->DR; 
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
}

/** 
* @brief: Writes data to a register
* @param: reg register to write to
* @param: values array of uint8_t data to be transmitted
* @param: num_bytes number of bytes to be transmitted
*/
void nrf24_multiwrite_register(uint8_t reg, uint8_t *values, uint8_t num_bytes) {
    uint8_t txData[num_bytes+1]; // Transmit data buffer

    // Prepare command to write (register address with write command prefix)
    txData[0] = reg | WRITE_COMMAND; // Write command
    memcpy(&txData[1], values, num_bytes); // Copy data to write into buffer

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    // Start SPI transmission
    for (int i = 0; i < num_bytes+1; i++) {
      // Transmit byte
      *(__IO uint8_t*)(&SPI1->DR) = txData[i]; 

      // Wait until transmission is complete
      while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

      // Read received byte (not used, but necessary to complete the transaction) 
      (void)SPI1->DR; 
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
}

void nrf24_write_command(uint8_t COMMAND) {
  // Set CSN low to start communication
  set_nrf24_SPI_CSN(0);
    
    // Start SPI transmission
	// Transmit byte
	*(__IO uint8_t*)(&SPI1->DR) = COMMAND; 

	// Wait until transmission is complete
	while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

	// Read received byte (not used, but necessary to complete the transaction) 
	(void)SPI1->DR; 

    // Set CSN high to end communication
  set_nrf24_SPI_CSN(1);
}

uint8_t nrf24_read_register(uint8_t reg) {
    uint8_t txData[2]; // Transmit data buffer
    uint8_t rxData[2]; // Receive data buffer

    // Prepare command to read (register address with read command bit)
    txData[0] = reg | READ_COMMAND; // Read command 
    txData[1] = 0x00; // Dummy byte for clocking out data

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    // Start SPI transmission and reception
    for (int i = 0; i < 2; i++) {
        // Transmit byte
        *(__IO uint8_t*)(&SPI1->DR) = txData[i];

        // Wait until transmission is complete
        while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

        // Read received byte
        rxData[i] = SPI1->DR; // Read received data
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
    return rxData[1]; // Return the value read from the register
}

uint8_t * nrf24_multiread_register(uint8_t reg, uint8_t num_bytes, uint8_t *rxData) {
    uint8_t txData; // Transmit data 

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    // Start SPI transmission and reception
    for (int i = 0; i < num_bytes + 1; i++) {
        // Transmit byte
		txData = (i == 0) ? (reg | READ_COMMAND) : 0x00;
        *(__IO uint8_t*)(&SPI1->DR) = txData;

        // Wait until transmission is complete
        while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

        // Read received byte
        rxData[i] = SPI1->DR; // Read received data
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
    return &rxData[1]; // Return the value read from the register TODO fix this pointer so it can't dangle by passing in array from calling function
}

/** 
* @brief: Writes data to the TX FIFO
* @param: value: Array of data to be transmitted
* @param: ack: 1 if the data is to be acknowledged, 0 if not
* @param: len: length of the data to be transmitted
*/
void nrf24_write_TX_payload(uint8_t * value, int ack, int len) {
    uint8_t txData[len+1]; // Transmit data buffer

    // Prepare command to write (register address with write command prefix)
    txData[0] = ack ?  WRITE_PAYLOAD_COMMAND: WRITE_PAYLOAD_NOACK; // Write command
    for (int i = 0; i < len; i++) {
        txData[i+1] = value[i];
    }
    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    // Start SPI transmission
    for (int i = 0; i < (len+1); i++) {
        // Transmit byte
        while (!(SPI1->SR & SPI_SR_TXE)); 
        *(__IO uint8_t*)(&SPI1->DR) = txData[i]; 

        // Wait until transmission is complete
        
        while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

        // Read received byte (not used, but necessary to complete the transaction) 
        (void)SPI1->DR; 
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
}

/** 
* @brief: Clear TX FIFO
*/
void nrf24_clear_TX(){
    uint8_t txData[1]; // Transmit data buffer

    // Prepare command to write (register address with write command prefix)
    txData[0] = FLUSH_TX; // Write command

    // Set CSN low to start communication
    set_nrf24_SPI_CSN(0);
    
    // Start SPI transmission
    for (int i = 0; i < 1; i++) {
        // Transmit byte
        *(__IO uint8_t*)(&SPI1->DR) = txData[i]; 

        // Wait until transmission is complete
        while (!(SPI1->SR & SPI_SR_RXNE)); // Wait until receive buffer is not empty

        // Read received byte (not used, but necessary to complete the transaction) 
        (void)SPI1->DR; 
    }

    // Set CSN high to end communication
    set_nrf24_SPI_CSN(1);
}

void test_nrf24_connection() {
    // int success = 0;
    set_nrf24_SPI_CSN(1); //make sure these pins are at the right level
    set_nrf24_SPI_CE(0);
    delay_microseconds(100000, NULL); //Let the chip power up and down
    /*
    uint8_t configValue = nrf24_read_register(STATUS_REG); 
    nrf24_write_register(STATUS_REG, CONFIG_SETTINGS); 
    
    uint8_t configValue2 = nrf24_read_register(STATUS_REG); 

    //set_nrf24_SPI_CE(1); //enables chip to receive data
    //Delay(2);

    // Check if expected bits are set
    if ((configValue & CONFIG_SETTINGS) == CONFIG_SETTINGS) {
        success = 1;
    }*/ 
}

uint8_t ADDRESS_LEN = 3;
/** 
* @brief: transmits data for testing purposes
* @param: data: array of data to be transmitted, up to 32 bytes long
* @param: data_len: length of the data to be transmitted
*/
 //TODO make this much more functional
void transmitBytesNRF(uint8_t * data, uint8_t data_len) {
    uint8_t write_address [3] = {0x93, 0xBD, 0x6B};
    
    // Clear TX FIFO
    nrf24_clear_TX();
    nrf24_write_register(STATUS_REG, 0x30); // Clear MAX_RT and TX Data Sent bit from status register
    nrf24_write_register(ENAA, 0x01); // Enable auto ack for pipe 0 //ALL PIPES 0x3F

    // Set control registers
    nrf24_write_register(SETUP_AW, 0x01); // Set to 3 byte address width
    nrf24_multiwrite_register(TX_ADDR, write_address, ADDRESS_LEN); // Set write address
    nrf24_multiwrite_register(RX_ADDR_P0, write_address, ADDRESS_LEN); // Set read address
    //nrf24_write_register(RF_SETUP, 0x00); // Set RF Data Rate to 1Mbps, RF output power to -18dBm
    //CHANGE POWER TO MAX 0dBm TEST
    nrf24_write_register(RF_SETUP, 0x07);
    nrf24_write_register(RX_PW_P0, 0x20); // Set payload size to 32 bytes
    
    nrf24_write_register(FEATURE, 0x01); // Enable W_TX_PAYLOAD_NOACK command

    nrf24_write_TX_payload(data, ACK, data_len);            // Write data to be transmitted into TX FIFO
    set_nrf24_SPI_CE(1);                  // Enable chip to transmit data
    delay_microseconds(130, NULL); // Wait for chip to go into TX mode
    delay_microseconds(15, NULL);    // Not sure how long this delay needs to be TODO test this

    set_nrf24_SPI_CE(0); // Disable chip after transmission
}

/** 
* @brief: transmits data for testing purposes
* @param: data: array of data to be transmitted
* @param: data_len: length of the data array to be transmitted in units of data size
* @param: data_size: size of the data type to be transmitted in bytes
*/
void transmit(void * data, uint8_t data_len, uint8_t data_size){ 
  // Data_size must divide data_len and 32 without a remainder and be at least 1
  if (data_len % data_size != 0 || 32 % data_size != 0 || data_size < 1){
    return;
  }
  int i = 0;
  int len_left = 0;
  uint8_t data_seg[32];
  //uint8_t data_send[32];
  nrf24_write_register(CONFIG, 0x0A);         // Set to PTX mode and turn on power bit 0x0A
  delay_microseconds(2*1000, NULL);  // Wait for chip to go into Standby-I mode
  
  while(data_len > 0){
    len_left = ((data_len*data_size) >= 32) ? 32 : (data_len*data_size)%32; 
    memcpy(&data_seg[0], (const unsigned char *)data + i, len_left); // Mini array of length 32 for buffering transmitted data

    transmitBytesNRF(data_seg, len_left);

    //while(!(SPI1->SR & ((uint16_t)(1 << 5)))); // Wait for TX_DS bit to be set from ACK received// TODO don't know if this is working
    if (data_len * data_size > 32) {
      data_len -= 32 / data_size;
    } else {
      data_len = 0; 
    }

    i+=32/data_size;
  }
  

  //
  uint16_t bytes_left = data_len * data_size;
  uint8_t* ptr = data;

  while (bytes_left > 0) {
      uint8_t chunk = bytes_left >= 32 ? 32 : bytes_left;
      memcpy(data_seg, ptr, chunk);

      transmitBytesNRF(data_seg, chunk);

      ptr += chunk;
      bytes_left -= chunk;
  }
}

/**
 * @brief  Controls the CSN pin for the NRF24LO1+ module. Active low
 * @retval None
 */
void set_nrf24_SPI_CSN(uint8_t input){
  while (SPI1->SR & SPI_SR_BSY); // Wait until SPI is not busy
  if(input == 1){
    (GPIOA->BSRR = GPIO_BSRR_BS_4);
  }
  else{
    delay_microseconds(5, NULL); // Makes sure CSN doesn't go low too quickly after last SPI operation
    (GPIOA->BSRR = GPIO_BSRR_BR_4);
  }
}

/**
 * @brief  Enables the CE pin for the NRF24LO1+ module. Active low
 * @retval None
 */
void set_nrf24_SPI_CE(uint8_t input){
  if(input == 1){
    (GPIOA->BSRR = GPIO_BSRR_BS_9);
  }
  else{
    delay_microseconds(5, NULL); // Makes sure CE doesn't go low too quickly after last SPI operation TODO check if this is necessary
    (GPIOA->BSRR = GPIO_BSRR_BR_9);
  }
}

/**
 * @brief Initializes the SPI connection for the STM32F030R8
 * @retval None
 */
void MySPI_Init(){
  //Set up the SPI peripheral
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // TODO CHECK IF THIS IS ALREADY ENABLED
  GPIO_InitTypeDef GPIO_InitStruct;
  SPI_InitTypeDef SPI_InitStruct;
  
  //GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);

  //Set up the SPI pins
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  //Set up the SPI peripheral
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;

  SPI1->CR2 |= SPI_CR2_FRXTH; // RXNE event is generated if the FIFO level is greater than or equal to 8

  //Initialize the SPI peripheral
  SPI_Init(SPI1, &SPI_InitStruct);
  SPI_Cmd(SPI1, ENABLE);
}

/** 
 * @brief Initialize the NRF24L01+ module
 * @retval None
*/
void NRF24L01p_Init(){
  //Set up the SPI peripheral
  MySPI_Init();

  //Set up the GPIO pins
  GPIO_InitTypeDef GPIO_InitStruct_1;
  //Set up the CSN pin
  GPIO_InitStruct_1.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStruct_1.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct_1.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct_1.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct_1.GPIO_PuPd = GPIO_PuPd_DOWN; 
  GPIO_Init(GPIOA, &GPIO_InitStruct_1);

  GPIO_InitTypeDef GPIO_InitStruct_2;
  //Set up the IRQ pin
  GPIO_InitStruct_2.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStruct_2.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct_2.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct_2.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct_2.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStruct_2);

   GPIO_InitTypeDef GPIO_InitStruct_3;
  //Set up the CE pin
  GPIO_InitStruct_3.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStruct_3.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct_3.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct_3.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct_3.GPIO_PuPd = GPIO_PuPd_DOWN; 
  GPIO_Init(GPIOA, &GPIO_InitStruct_3);
}

/**
 * @brief Initializes the UART connection for the STM32F030R8
 * @retval None
 */
void UART_Settings_Init(){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 
  GPIO_Init(GPIOA, &GPIO_InitStruct);


  USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.USART_BaudRate = 9600;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_Mode = USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART2, &USART_InitStruct);
  //No synchronous mode yet
  USART_Cmd(USART2, ENABLE);
}

  /**
   * @brief Sets up ICM20948 for data collection and puts magnetometer(AK09916) into continuous mode 4 (100 Hz data rate)
   * @param None
   * @retval N0ne
   */
  void ICM20948_init(){
    
    I2C_transmit(ICM_20948_I2C_ADDRESS, REG_BANK_SEL, USER_BANK_0); // switch to USER BANK 0

    I2C_transmit(ICM_20948_I2C_ADDRESS, LP_CONFIG, 0x30); // set low power mode for ICM20948 (ACCEL/ GYRO in duty cycle mode)

    I2C_transmit(ICM_20948_I2C_ADDRESS, PWR_MGMT_1, 0x11); // set low power enable for digital circuitry (does not turn on LP mode, use LP_CONFIG) and auto clock select for ICM20948
    I2C_transmit(ICM_20948_I2C_ADDRESS, PWR_MGMT_2, 0x00); // enable accelerometer and gyroscope for ICM20948

    I2C_transmit(ICM_20948_I2C_ADDRESS, REG_BANK_SEL, USER_BANK_2); // switch to USER BANK 2
  
    I2C_transmit(ICM_20948_I2C_ADDRESS, TEMP_CONFIG, 0x01); // set TEMP_DLPCFG for temperature sensor to 1

    I2C_transmit(ICM_20948_I2C_ADDRESS, GYRO_SMPLRT_DIV, 0x07); // set gyro sample rate divider to 7 for ICM20948
    I2C_transmit(ICM_20948_I2C_ADDRESS, GYRO_CONFIG_1, 0x3F); // Set gyro +- range to 2000dps and DLPF 3dB point to 361 Hz, and enable LPF for ICM20948
    I2C_transmit(ICM_20948_I2C_ADDRESS, GYRO_CONFIG_2, 0x02); // Enable 4x averaging for gyro data for ICM20948

    I2C_transmit(ICM_20948_I2C_ADDRESS, ACCEL_SMPLRT_DIV, 0x0A); // set accel sample rate divider to 10 for ICM20948
    I2C_transmit(ICM_20948_I2C_ADDRESS, ACCEL_CONFIG, 0x3F); // Set accel +- range to 16g and DLPF 3dB point to 473 Hz, and enable LPF for ICM20948
    I2C_transmit(ICM_20948_I2C_ADDRESS, ACCEL_CONFIG_2, 0x00); // Enable 4x averaging for accel data for ICM20948

    I2C_transmit(ICM_20948_I2C_ADDRESS, REG_BANK_SEL, USER_BANK_0); // switch to USER BANK 0

    // Enable bypass mode for I2C access of internal magnetometer
    I2C_transmit(ICM_20948_I2C_ADDRESS, 0x0F, 0x02);  // INT_PIN_CFG: BYPASS_EN = 1

    I2C_transmit(0x0C, 0x31, 0x08);  // CNTL2 register = 0x16 (continuous 100 Hz updates to magnetometer data)
    delay_microseconds(10*1000, NULL); // TODO find out how long the magnetometer needs to start up
  }

  /**
 * @brief Collects ambient data from ICM20948 and puts it into the provided array ICM_data
 * @param None
 * @retval None
 */
  void getICM20948_ACCEL_GYRO_TEMPdata(float * ICM_data){
    static float ACCEL_SENSITIVITY = 2048;
    static float GYRO_SENSITIVITY = 16.4; 
    static float MAG_SENSITIVITY = 0.15;
    static float TEMP_SENSITIVITY = 337.87; 

    // Accelerometer and Gyroscope variables
    uint8_t accel_gyro_data[14];
    float accel_out[3]; //{X,Y,Z}
    float gyro_out[3]; //{X,Y,Z}

    // Magnetometer variables
    int16_t mag_raw[3];
    float mag_out[3];
    uint8_t buf[6];
    uint8_t new_magdata = 0;
    uint8_t mag_overf = 0;
    uint8_t st1 = 0;
    uint8_t st2 = 0; 

      // Read accelerometer and gyroscope data test
    I2C_receive(ICM_20948_I2C_ADDRESS, GYRO_ACCEL_START_REG, accel_gyro_data, 14); // read ACCEL_XOUT_H register and the following 11 registers for accel and gyro data

    // Thermometer variables
    float temp_out = (((accel_gyro_data[12] << 8) + accel_gyro_data[13] - 0) / TEMP_SENSITIVITY) + 21; // Degrees Celcius

    accel_out[0] = ((accel_gyro_data[0] << 8) + accel_gyro_data[1]) / ACCEL_SENSITIVITY * 9.81; // TODO adjust gyro and accelerometer data for temperature drift values
    accel_out[1] = ((accel_gyro_data[2] << 8) + accel_gyro_data[3]) / ACCEL_SENSITIVITY * 9.81; // m/s^2 // TODO reading very high m(309.6 m/s^2), bug?
    accel_out[2] = ((accel_gyro_data[4] << 8) + accel_gyro_data[5]) / ACCEL_SENSITIVITY * 9.81; // m/s^2

    gyro_out[0] = (accel_gyro_data[6] * UINT8_MAX + accel_gyro_data[7]) / GYRO_SENSITIVITY; // Degrees per second
    gyro_out[1] = (accel_gyro_data[8] * UINT8_MAX + accel_gyro_data[9]) / GYRO_SENSITIVITY; // Degrees per second
    gyro_out[2] = (accel_gyro_data[10] * UINT8_MAX + accel_gyro_data[11]) / GYRO_SENSITIVITY; // Degrees per second

    temp_out = (((accel_gyro_data[12] << 8) + accel_gyro_data[13] - 0) / TEMP_SENSITIVITY) + 21; // Degrees Celcius

    // BANK 0: Enable I2C master
    // Should still be in BANK 0
    
    do {
      I2C_receive(0x0C, 0x10, &st1, 1);  // read ST1
    } while (!(st1 & 0x01)); // Wait for DRDY flag
    I2C_receive(0x0C, MAG_START_REG, buf, 6); // 0x0C I2C address is for the internal magnetometer
    I2C_receive(0x0C, 0x18, &st2, 1);  // read ST2 // TODO handle overflow flags here 

    // Now interpret:
    mag_raw[0] = (int16_t)((buf[1] << 8) | buf[0]); // X
    mag_raw[1] = (int16_t)((buf[3] << 8) | buf[2]); // Y
    mag_raw[2] = (int16_t)((buf[5] << 8) | buf[4]); // Z

    // ST1 check (data ready)
    if(!(st1 & 0x01)) {
        // TOD0 No new data; skip using this sample
        new_magdata = 0; 
    }

    // ST2 check (overflow)
    if(st2 & 0x08) {
        // TODO Magnetic overflow — discard values
        mag_overf = 1;
    }

    mag_out[0] = mag_raw[0] * MAG_SENSITIVITY; // uT
    mag_out[1] = mag_raw[1] * MAG_SENSITIVITY; // uT
    mag_out[2] = mag_raw[2] * MAG_SENSITIVITY; // uT

    ICM_data[0] = accel_out[0];
    ICM_data[1] = accel_out[1];
    ICM_data[2] = accel_out[2];
    ICM_data[3] = gyro_out[0];
    ICM_data[4] = gyro_out[1];
    ICM_data[5] = gyro_out[2];
    ICM_data[6] = temp_out;
    ICM_data[7] = mag_out[0];
    ICM_data[8] = mag_out[1];
    ICM_data[9] = mag_out[2];
  }