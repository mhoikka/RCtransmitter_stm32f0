#include <stdio.h>
#include "stm32f0xx_conf.h"
#include "stm32f0xx_i2c.h"
#include "main.h"
#include "bme280.h"
#include "bme280_defs.h"
#include "stm32f0xx_spi.h"
#include "Peripheral_Init.h"

const uint8_t DEGREE_45 = 1;

void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

static __IO uint32_t TimingDelay;
uint8_t __IO BlinkSpeed = 0;

RCC_ClocksTypeDef RCC_Clocks;

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f030.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */ 
  /* SysTick end of count event each 1ms */

  unsigned char data[32] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                            0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                            0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                            0x07, 0x08, 0x09, 0x01, 0x02, 0x03, 0x04, 0x05};
  uint8_t control_packet[32] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // Each index in this adc array corresponds to the ADC channel that the data is drawn from on the STM32F030R8T6 used in this project
  uint16_t adc[15] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t adc_8bit_values[4] = {0x00, 0x00, 0x00, 0x00};
  uint16_t adc_controller_values[4] = {0x00, 0x00, 0x00, 0x00};
  uint16_t *sensor_data[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint16_t *predictions[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t servos = 0x00; // Each bit corresponds to one servo control signal, // TODO read control signals and construct this value
  uint8_t checksum = 0x00; // leave this for now // TODO implement checksum calculation for control_packet

  System_Clock_Init();

  // Servo_Peripherals_Init();
  // DriveServoControl(DEGREE_45);

  // HBridge_Peripherals_Init();
  // DriveACMotorVoltageController(4500); 

  //ADCPeripherals_Init();
  // uint16_t adc_values[3];
  // ADC_take_Readings();
  //uint16_t adc_battery_reading = 0;
  //adc_battery_reading = ADC_take_BatteryReading(adc_battery_reading);
  //volatile float battery_voltage = 0.0f;
  //battery_voltage = batteryVoltageMeasurement(adc_battery_reading);

  I2C_Settings_Init();

  float ambient_data[10];
  
  ICM20948_init();
  getICM20948_ACCEL_GYRO_TEMPdata(ambient_data);
  getICM20948_ACCEL_GYRO_TEMPdata(ambient_data);

  
  NRF24L01p_Init(); // TEST

  Delay(1);
  test_nrf24_connection(); // TEST
  delay_microseconds(100*1000, NULL);  // Wait for NRF24L01+ to power on TEST

  //transmit(adc_battery_reading, sizeof(data)/sizeof(unsigned char), sizeof(unsigned char)); // TEST
 
  while (1)
  {
    // TODO receive sensor data from RC_Receiver on plane
    ADC_take_Multiple_Readings(0x000F, adc); // Use analog inputs A0-A3, first three entries in adc array
    ADC_convert_to_8bit_controls(adc_controller_values, adc_8bit_values); 
    generate_predictions(sensor_data, predictions, 10); // TODO read sensor data
    makeControlSignal(adc_8bit_values, servos, checksum, predictions, control_packet);
    transmit(control_packet, sizeof(control_packet), sizeof(uint8_t)); // TEST Maximum 350 Hz with 1Mbps transmissions. Should be more than good enough.
    Delay(1000);
  }
}
 
/**
 * @brief  Initializes the STM32F030R8 clock
 * @retval None
 */
void System_Clock_Init(){
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000); // TODO is this millisecond timer accurate? It seems like it would run 8x too fast
}

/**
* @brief  Inserts a delay time.
* @param  nTime: specifies the delay time length, in ms.
* @retval None
*/
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
* @brief  Decrements the TimingDelay variable.
* @param  None
* @retval None
*/
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

/**
* @brief  function to reverse array
* @param  buffer: array to be reversed
* @param  i: start index
* @param  j: end index
* @retval pointer to reversed char array
*/
char* reverse(char *buffer, int i, int j)
{
	while (i < j)
		swap(&buffer[i++], &buffer[j--]); 
	return buffer;
}

/**
* @brief  Iterative function to implement itoa() function in C
* @param  value: value of integer to convert
* @param  buffer: buffer to store the result
* @param  base: base of the result
* @retval pointer to converted character array
*/
char* itoa(int value, char* buffer, int base)
{
	// invalid input
	if (base < 2 || base > 32)
		return buffer; 

	// Get absolute value of number
	int n = value;
    if (n < 0) n *= -1;

	int i = 0;
	while (n)
	{
		int r = n % base;

		if (r >= 10) 
			buffer[i++] = 65 + (r - 10);
		else
			buffer[i++] = 48 + r;

		n = n / base;
	}

	// if number is 0
	if (i == 0)
		buffer[i++] = '0';

	// If base is 10 and value is negative, the resulting string 
	// is preceded with a minus sign (-)
	// With any other base, value is always considered unsigned
	if (value < 0 && base == 10)
		buffer[i++] = '-';

	buffer[i] = '\0'; // null terminate string

	// reverse the string and return it
	return reverse(buffer, 0, i - 1);
}

// We're just going to block for now.
// This is probably never going to DMA
/**
* @brief  Sends a string to over UART 
* @param  string: String to send
* @retval None
*/
void send_string(char *string)
{
    while (*string != 0)
    {
        while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
        USART_SendData(USART2, (uint16_t) *string++);
    }
    while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
}

/**
* @brief  Sends a string to over UART with a newline character
* @param  string: String to send
* @retval None
*/
void send_stringln(char *string)
{
  send_string(string);
  send_string("\r\n");
}

#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add their own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/