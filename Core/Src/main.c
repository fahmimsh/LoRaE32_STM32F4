/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

#include "math.h"
#include "INA219.h"
#include "retarget.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "GPS.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
//serial 2
#define RxBuf_SIZE	  1024
#define MainBuf_SIZE  1024
uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];
//serial 3
uint8_t MainBuf_3[MainBuf_SIZE] = { 0 };
uint8_t RxBuf_3[RxBuf_SIZE] = { 0 };
bool usart_3_state = false;

//serial komunikasi
uint8_t buff_s[200];
uint16_t ukuranstring;
char buffer[5];
uint16_t oldPos = 0;
uint16_t newPos = 0;
uint16_t oldPos_3 = 0;
uint16_t newPos_3 = 0;
//waktu millis second
char jam, menit, detik;
char tanggal, bulan, tahun;
char gabungtanggal[50];
unsigned long rtc_millis = 0;
unsigned long ina219_millis = 0;
unsigned long led_prev_on = 0;
unsigned long led_loop_on = 0;
//deklarasi Sensor Tegangan dan Arus
float tegangan = 0.0;
float Vshunt = 0.0;
float arus = 0.0;
float batteryPercentage = 0.0;
char volt[50], persen[50], amper[50];
//deklarasi keypad
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint8_t keyPressed = 0;
uint8_t keyPressed_prev = 0;
uint8_t counter = 0;
char keypad = 0; char key[5] = {0};
bool key_kondisi = false;
//deklarasi LED
bool led_hijau_kuning = 1;
bool led_merah = 0;
//deklarasi GPS
double lon_gps, lat_gps;
char lat[20], lat_a, lon[20], lon_a;
char data_lat[20], data_lng[20];
//deklarasi LCD
char data[] = "HELLO WORLD \r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long int tick = 29380;

char printbuffer[300];

bool printed;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(&huart3, Rx_data, RX_BUFFER_SIZE);
  GPS.message=Rx_data;
  update = true;
}
/**Fungsi ini digunakan untuk set timer jika dibutuhkan untuk set rtc
  */
void set_time(void){
	  RTC_TimeTypeDef sTime = {0};
	  RTC_DateTypeDef sDate = {0};
	  /** Initialize RTC and set the Time and Date
	  */
	  sTime.Hours = 0x10;
	  sTime.Minutes = 0x1;
	  sTime.Seconds = 0x0;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	  sDate.Month = RTC_MONTH_OCTOBER;
	  sDate.Date = 0x18;
	  sDate.Year = 0x21;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN RTC_Init 2 */
	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
	  /* USER CODE END RTC_Init 2 */

}
/**Fungsi ini digunakan untuk LCD*/

void getLCD(char *data_lat,char *data_lng, char *volt, char *amper, char *persen, char *gabungtanggal)
{
	  ILI9341_DrawVLine(0, 0, 320, DARKGREEN);
	  ILI9341_DrawVLine(2, 0, 320, DARKGREEN);
	  ILI9341_DrawText(gabungtanggal, FONT3, 9, 8, WHITE, BLACK);
	  ILI9341_DrawText(persen, FONT3, 155, 8, WHITE, BLACK);
	  if(atoi(persen) <= 100 && atoi(persen) >= 80)
	  {
		  ILI9341_DrawRectangle(190, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(200, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(210, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(220, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(230, 4, 8, 20, GREENYELLOW);
	  }
	  else if(atoi(persen) <= 60 && atoi(persen) >= 79)
	  {
		  ILI9341_DrawRectangle(200, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(210, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(220, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(230, 4, 8, 20, GREENYELLOW);
	  }
	  else if(atoi(persen) <= 60 && atoi(persen) >= 79)
	  {
		  ILI9341_DrawRectangle(210, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(220, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(230, 4, 8, 20, GREENYELLOW);
	  }
	  else if(atoi(persen) <= 60 && atoi(persen) >= 79)
	  {
		  ILI9341_DrawRectangle(220, 4, 8, 20, GREENYELLOW);
		  ILI9341_DrawRectangle(230, 4, 8, 20, GREENYELLOW);
	  }
	  else
	  {
		  ILI9341_DrawRectangle(230, 4, 8, 20, GREENYELLOW);
	  }
	  ILI9341_DrawVLine(237, 0, 320, DARKGREEN);
	  ILI9341_DrawVLine(239, 0, 320, DARKGREEN);
	  ILI9341_DrawHLine(0, 30, 240, DARKGREEN);
	  ILI9341_DrawHLine(0, 32, 240, DARKGREEN);
	  ILI9341_DrawText("ID NODE : A", FONT3, 9, 40, WHITE, BLACK);
	  ILI9341_DrawText(volt, FONT3, 120, 40, WHITE, BLACK);
	  ILI9341_DrawText(amper, FONT3, 180, 40, WHITE, BLACK);
	  ILI9341_DrawHLine(0, 60, 240, DARKGREEN);
	  ILI9341_DrawHLine(0, 62, 240, DARKGREEN);
	  ILI9341_DrawText("Masukkan Pesan yang anda kirim : ", FONT2, 9, 70, WHITE, BLACK);
	  ILI9341_DrawHLine(0, 85, 240, DARKGREEN);
	  ILI9341_DrawHLine(0, 87, 240, DARKGREEN);
	  ILI9341_DrawText("Ini Pesan Anda", FONT2, 9, 95, WHITE, BLACK);
	  ILI9341_DrawHLine(0, 150, 240, DARKGREEN);
	  ILI9341_DrawHLine(0, 152, 240, DARKGREEN);
	  ILI9341_DrawText("Anda Menerima Pesan : ", FONT2, 9, 160, WHITE, BLACK);
	  ILI9341_DrawText("Ini Pesan dia", FONT2, 9, 175, WHITE, BLACK);
	  ILI9341_DrawHLine(0, 245, 240, DARKGREEN);
	  ILI9341_DrawHLine(0, 247, 240, DARKGREEN);
	  ILI9341_DrawText("Lokasi   : ", FONT2, 9, 255, WHITE, BLACK);
	  ILI9341_DrawText("Latitude : ", FONT2, 9, 270, WHITE, BLACK);
	  ILI9341_DrawText(data_lat, FONT2, 75, 270, WHITE, BLACK);
	  ILI9341_DrawText("Longitude : ", FONT2, 9, 285, WHITE, BLACK);
	  ILI9341_DrawText(data_lng, FONT2, 75, 285, WHITE, BLACK);
	  ILI9341_DrawHLine(0, 300, 240, DARKGREEN);
	  ILI9341_DrawHLine(0, 300, 240, DARKGREEN);
	  ILI9341_DrawText("Copyright : www.pens.ac.id", FONT2, 30, 305, WHITE, BLACK);
}

/**Fungsi ini digunakan untuk membaca interanal RTC
  */
void get_time(void)
{
	if (HAL_GetTick() - rtc_millis >= 500){
		rtc_millis = HAL_GetTick();
		 RTC_DateTypeDef gDate;
		 RTC_TimeTypeDef gTime;
		 HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
		 HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
		 jam = gTime.Hours; menit = gTime.Minutes; detik = gTime.Seconds;
		 tanggal = gDate.Date; bulan = gDate.Month; tahun = gDate.Year;
//		 printf("%02d:%02d:%02d || %02d-%02d-%2d\r\n",jam, menit, detik,tanggal, bulan, 2000 + tahun);
	}
}
/**Fungsi ini digunakan untuk membaca baterai
  */
void get_ampere_volt(void){
	setCalibration_16V_400mA();
	tegangan = getBusVoltage_V();
	Vshunt = getPower_mW();
	arus = getCurrent_mA() * (-1);
	float maxVoltage = 12.4;
	float minVoltage = 11.2;
	batteryPercentage = (tegangan - minVoltage) / (maxVoltage - minVoltage) * 100;
	if (batteryPercentage > 100) batteryPercentage = 100;
	else if (batteryPercentage < 0) batteryPercentage = 0;
	if (HAL_GetTick() - ina219_millis >= 500){
		ina219_millis = HAL_GetTick();
//		printf("Vbus: %.1f V| persen: %.1f percent | Ampere: %.1f mA\r\n",tegangan, batteryPercentage, arus);
		sprintf(volt, "%.4g", tegangan);
		sprintf(amper, "%.5g", arus);
		sprintf(persen, "%.2g", batteryPercentage);
//
	}
}
void get_keypad(uint8_t keypadin);
void get_gps();
void led_reaction(uint16_t led_time, uint16_t time_loop);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RxBuf_3, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  //set m0 m1
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 0);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, 0);
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2){
	  set_time();
  }
  //begin LCD
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
  ILI9341_Init();
  ILI9341_FillScreen(BLACK);
  ILI9341_SetRotation(SCREEN_VERTICAL_2);
  //begin keypad
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1);
  //Millis second set
  rtc_millis = HAL_GetTick();
  ina219_millis = HAL_GetTick();
  //kirim data dummy  *node,lat,long,data,jam, menit, detik, tegangan
  ukuranstring = sprintf((char*)buff_s, "*A,latitude,longitude,data,jam,menit,detik,baterai\r\n");
  HAL_UART_Transmit(&huart2, buff_s, ukuranstring, 100);
  led_reaction(100, 600);

  //GPS
  HAL_UART_Receive_IT (&huart3, Rx_data, RX_BUFFER_SIZE);
   uint8_t ubxcfgrate[] = { // UBX-CFG-RATE 10 Hz Measurement/Navigation

   0xB5,0x62,0x06,0x08,0x06,0x00,

   0x64,0x00,0x01,0x00,0x01,0x00, // Payload

   0x7A,0x12 }; // Checksum
   HAL_UART_Transmit(&huart1,(uint8_t *)ubxcfgrate,14,200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  get_time();
	  get_keypad(keyPressed);
	  get_ampere_volt();
//	  jam, menit, detik,tanggal, bulan, 2000 + tahun
	  snprintf( gabungtanggal, 50, "%02d:%02d:%02d %02d-%02d-%2d",jam, menit, detik,tanggal, bulan, 2000 + tahun );
//	  printf("%s\n", gabungtanggal);
	  getLCD(lat, lon, volt, amper, persen,gabungtanggal);
//	  printf("%02d:%02d:%02d || %02d-%02d-%2d\r\n",jam, menit, detik,tanggal, bulan, 2000 + tahun);
//	  printf("%s", lat);
//	  get_gps();
	  printf("%lf",GPS.lat);
	   if(update){
	  		  update=false;
	  		  GPS_p();
	  		  printed=false;
	  	  }
	  	  long int test;
	      test = HAL_GetTick();                                                        //Hole Zeit von Systicktimer
	      if(!((GPS.FIXTIME+1000) >test)){GPS.valid = false;}                          //Fix verfällt noch 1 sekunde

	    if(GPS.valid&& !printed){
//	    	printf("%d:%d:%d.%d \n %lf  %c\n %lf  %c\n", GPS.hours,GPS.mins,GPS.secs,GPS.millis,GPS.latitude,GPS.NS_ind, GPS.longitude,GPS.EW_ind);
	    	sprintf(printbuffer,"%d:%d:%d.%d \n latitude:%lf NS_ind: %c\n longitude:%lf SE_ind: %c\nHDOP : %lf\nSats : %d\nAltitude:  %lf M\n", GPS.hour,GPS.min,GPS.sec,GPS.millis,GPS.lat,GPS.NS_ind, GPS.lon,GPS.EW_ind,GPS.hdop,GPS.sats,GPS.alt);
	    	//HAL_UART_Transmit(&huart2,(uint8_t *)GPS.message,strlen(GPS.message),1000);
	    	printf("%lf",GPS.lat);
	    	HAL_UART_Transmit(&huart1,(uint8_t *) printbuffer,strlen(printbuffer),100);
	        sprintf(printbuffer,"indexbegin: %d       indexend:  %d\nChecksum: 0x%x    Berechnete Checksum: 0x%x\n",GPS.index_begin,GPS.index_end,GPS.checksum,GPS.checksum_calc);
	        HAL_UART_Transmit(&huart1,(uint8_t*) printbuffer,strlen(printbuffer),200);
	        printed = true;
	    	switch (GPS.fixtype){
	        	case 0:
	        	////printf("\n no fix\n");
	       	 	break;
	        	case 1:
	        	////printf("\n 2D fix\n");
	        	break;
	        	case 2:
	       		////printf("\n 3D fix\n");
	        	break;
	        default:
	        	////printf ("ERROR\n");
	        	HAL_Delay(100);
	    	}

	    }
	    else{
		    ////printf("NO VALID GPS DATA FOUND");
		}



//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
	//JANGAN LUPA DIKOMEN YA
  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x11;
  sTime.Minutes = 0x35;
  sTime.Seconds = 0x5;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_OCTOBER;
  sDate.Date = 0x18;
  sDate.Year = 0x21;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC2 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE10 PE12 PE14
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/**Fungsi ini digunakan untuk interrupt LoRa E32 jika ada data masuk
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//variable parse
	uint8_t new_Data[10];
	//uint8_t prev_Data;
	if (huart->Instance == USART2)
	{
		oldPos = newPos;  // Update the last position before copying new data

		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
		 * This is to maintain the circular buffer
		 * The old data in the main buffer will be overlapped
		 */
		if (oldPos+Size > MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy = MainBuf_SIZE-oldPos;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)MainBuf+oldPos, RxBuf, datatocopy);  // copy data in that remaining space

			oldPos = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)MainBuf, (uint8_t *)RxBuf+datatocopy, (Size-datatocopy));  // copy the remaining data
			newPos = (Size-datatocopy);  // update the position
		}

		/* if the current position + new data size is less than the main buffer
		 * we will simply copy the data into the buffer and update the position
		 */
		else
		{
			memcpy ((uint8_t *)MainBuf+newPos, RxBuf, Size);
			newPos = Size+oldPos;
		}
		/* start the DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) RxBuf, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

		if(RxBuf_3[0] == '*'){ //*node,lat,long,data,jam, menit, detik, tegangan
			new_Data[0] = MainBuf[1];
			if (new_Data[0]){
				HAL_UART_Transmit(&huart2, MainBuf, Size, 100);
			}
			//prev_Data = new_Data[0];
		}
		printf("%s", MainBuf);
		led_reaction(100, 600);
	}else if(huart->Instance == USART3){
		oldPos_3 = newPos_3;
		if (oldPos_3+Size > MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
		{
			uint16_t datatocopy_3 = MainBuf_SIZE-oldPos_3;  // find out how much space is left in the main buffer
			memcpy ((uint8_t *)MainBuf_3+oldPos_3, RxBuf_3, datatocopy_3);  // copy data in that remaining space
			oldPos_3 = 0;  // point to the start of the buffer
			memcpy ((uint8_t *)MainBuf_3, (uint8_t *)RxBuf_3+datatocopy_3, (Size-datatocopy_3));  // copy the remaining data
			newPos_3 = (Size-datatocopy_3);
		}
		else
		{
			memcpy ((uint8_t *)MainBuf+newPos_3, RxBuf_3, Size);
			newPos_3 = Size+oldPos_3;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) RxBuf_3, RxBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
//		printf("%s", MainBuf_3); //gpsdata
		usart_3_state = true;
	}
}
/**Fungsi ini digunakan untuk keypad untuk mengirim pesan dan juga di intrupsi jika ada data masuk dari keypad
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  currentMillis = HAL_GetTick();
  if (currentMillis - previousMillis > 10) {
    /*Configure GPIO pins : PB3 PB5 PB8 PB9 to GPIO_INPUT*/
    GPIO_InitStructPrivate.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 0);
    if(GPIO_Pin == GPIO_PIN_3 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
    {
      keyPressed = 14; //ASCII value of D
    }
    else if(GPIO_Pin == GPIO_PIN_5 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
    {
      keyPressed = 13; //ASCII value of C
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
    {
      keyPressed = 12; //ASCII value of B
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
    {
      keyPressed = 11; //ASCII value of A
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 0);

    if(GPIO_Pin == GPIO_PIN_3 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
    {
      keyPressed = 15; //ASCII value of #
    }
    else if(GPIO_Pin == GPIO_PIN_5 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
    {
      keyPressed = 9; //ASCII value of 9
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
    {
      keyPressed = 6; //ASCII value of 6
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
    {
      keyPressed = 3; //ASCII value of 3
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 0);
    if(GPIO_Pin == GPIO_PIN_3 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
    {
      keyPressed = 16; //ASCII value of 0
    }
    else if(GPIO_Pin == GPIO_PIN_5 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
    {
      keyPressed = 8; //ASCII value of 8
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
    {
      keyPressed = 5; //ASCII value of 5
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
    {
      keyPressed = 2; //ASCII value of 2
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1);
    if(GPIO_Pin == GPIO_PIN_3 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
    {
      keyPressed = 17; //ASCII value of *
    }
    else if(GPIO_Pin == GPIO_PIN_5 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
    {
      keyPressed = 7; //ASCII value of 7
    }
    else if(GPIO_Pin == GPIO_PIN_8 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
    {
      keyPressed = 4; //ASCII value of 4
    }
    else if(GPIO_Pin == GPIO_PIN_9 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9))
    {
      keyPressed = 1; //ASCII value of 1
    }
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, 1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, 1);
    /*Configure GPIO pins : PB6 PB7 PB8 PB9 back to EXTI*/
    GPIO_InitStructPrivate.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStructPrivate.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStructPrivate);
    //printf("nilai key pad %d\r\n", keyPressed);
    previousMillis = currentMillis;
    key_kondisi = true;
  }
}
/**Fungsi ini digunakan untuk mendapatkan karakter dari nilai keypad yangdigunakan
  */
void get_keypad(uint8_t keypadin){
	if(key_kondisi == true){
		if(keypadin != keyPressed_prev || counter >= 5){
	  	  counter = 0;
	    }
		if(keypadin == 1){
			counter ++;
			strcpy((uint8_t*)key, " 1abc");
		     keypad = key[counter];
		}else if(keypadin == 2){
			counter ++;
			strcpy((uint8_t*)key, " 2def");
		     keypad = key[counter];
		}else if(keypadin == 3){
			counter ++;
			strcpy((uint8_t*)key, " 3ghi");
		     keypad = key[counter];
		}else if(keypadin == 4){
			counter ++;
			strcpy((uint8_t*)key, " 4jkl");
		     keypad = key[counter];
		}else if(keypadin == 5){
			counter ++;
			strcpy((uint8_t*)key, " 5mno");
		     keypad = key[counter];
		}else if(keypadin == 6){
			counter ++;
			strcpy((uint8_t*)key, " 6pqr");
		     keypad = key[counter];
		}else if(keypadin == 7){
			counter ++;
			strcpy((uint8_t*)key, " 7stu");
		     keypad = key[counter];
		}else if(keypadin == 8){
			counter ++;
			strcpy((uint8_t*)key, " 8vwx");
		     keypad = key[counter];
		}else if(keypadin == 9){
			counter ++;
			strcpy((uint8_t*)key, " 4yz");
		     keypad = key[counter];
		}else if(keypadin == 11){
			counter = 0;
			strcpy((uint8_t*)key, "D"); //delete
		     keypad = key[counter];
		}else if(keypadin == 12){
			counter = 0;
			strcpy((uint8_t*)key, "O"); //oke / KIRIM
		     keypad = key[counter];
		}else if(keypadin == 13){
			counter = 0;
			strcpy((uint8_t*)key, " "); //spasi
		     keypad = key[counter];
		}
		printf("key %c\r\n", keypad);
		keyPressed_prev = keypadin;
		key_kondisi = false;
		led_reaction(100,100);
	  }
}
/**Fungsi ini LED Blink untuk indikasi
  */
void led_reaction(uint16_t led_time, uint16_t time_loop){
	led_loop_on = HAL_GetTick();
	while ((unsigned long)HAL_GetTick() - led_loop_on <= time_loop){
		if (HAL_GetTick() - led_prev_on >= led_time){
			led_prev_on = HAL_GetTick();
			if (led_hijau_kuning == 0){
				led_hijau_kuning = 1;
			} else {
				led_hijau_kuning = 0;
			}
			if (led_merah == 0){
				led_merah = 1;
			} else {
				led_merah = 0;
			}
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, led_hijau_kuning);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, led_merah);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, led_hijau_kuning);
		}
	}
}
/**Fungsi ini untuk memparsing data GPS yaitu latitude longitude digunakan untuk melihat lokaasi alat
  */
void get_gps(){
	if (usart_3_state == true){
		printf("%s", MainBuf_3);
//		printf("7.12122");
//		printf("test");
	  char *pointer; char *conv;
		  int length = sizeof(MainBuf_3);

		  memset(lat, '\0', 20);
		  memset(lon, '\0', 20) ;
		  pointer = strchr((char*)MainBuf_3, '$');
		  do{
			  char *ptrstart;
			  char *ptrend;
			  if(strncmp(pointer, "$GNGGA" , 6) == 0){ //$GNGGA
				  ptrstart = (char*)memchr(pointer + 1, ',', length);
				  ptrstart = (char*)memchr(ptrstart + 1, ',', length);
				  ptrend = (char*)memchr(ptrstart + 1, ',', length);

			  } else if(strncmp(pointer, "$GNGLL", 6) == 0){ //$GNGLL
				  ptrstart = (char*)memchr(pointer + 1, ',', length);
				  ptrend = (char*)memchr(ptrstart + 1, ',', length);

			  } else if(strncmp(pointer, "$GNRMC", 6) == 0){ //$GNRMC
				  ptrstart = (char*)memchr(pointer + 1, ',', length);
				  ptrstart = (char*)memchr(ptrstart + 1, ',', length);
				  ptrstart = (char*)memchr(ptrstart + 1, ',', length);
				  ptrend = (char*)memchr(ptrstart + 1, ',', length);

			  } else {
				  pointer = strchr(pointer + 6, '$');
				  continue;
			  }

			  for(int i = 1; i < (ptrend - ptrstart); i++) lat[i - 1] = ptrstart[i];
			  lat_a = *(ptrend + 1);

			  ptrstart = (char*)memchr(ptrend + 1, ',', length);
			  ptrend = (char*)memchr(ptrstart + 1, ',', length);

			  for(int i = 1; i < (ptrend - ptrstart); i++) lon[i - 1] = ptrstart[i];
			  lon_a = *(ptrend + 1);
			  if(lon[0] != '\0' && lat[0] != '\0'){
//				  printf("Lat: %s | %c\tLon: %s | %c\r\n", lat, lat_a, lon, lon_a);
				  lat_gps = strtod((char*)lat, &conv);
						  //atof((char*)lat);
				  lon_gps = strtod((char*)lon, &conv);
						  //atof((char*)lon);
				  sprintf(lat, "%g", lat_gps);
				  sprintf(lon, "%g", lon_gps);

				  break;
			  }

			  pointer = strchr(pointer + 4, '$');
		  }
		  while(pointer != NULL);
		  usart_3_state = false;
	  }
			else
			  {
				  sprintf(lat, "%g", -7.122323);
				  sprintf(lon, "%g", 122.32312);
			  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
