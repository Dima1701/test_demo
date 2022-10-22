/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "udp.h"
#include "string.h"
//#include "FLASH_SECTOR.h"
#include "pbuf.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STM_PORT 50008
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint16_t buffer[200] = { 0 };
/*receive data buffers*/
uint16_t rx_flash_buf[200] = { 0 };
/*transmit data buffers */
uint16_t tx_flash_buffer[200] = { 0 };
/*global variable to store original data from buffer and send back without "string" in data buffer*/
uint16_t data_send[200] = {0} ;
/*global variable for buffer length pushed from function udp_receive_callback */
uint8_t buf_len;

/** the UDP protocol control block */
struct udp_pcb *upcb;
/*flag who do trigger for udp_receive_callback function after get USER massage */
int udp_recv_data = 0;

struct netif gnetif;

ip_addr_t des_adder = { 0 };
u16_t des_port = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
/* USER CODE BEGIN PFP */
/*udp_receive_callback function is our receive callback.
 This callback will be called when a client
  sends some DATAto the server*/
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
		const ip_addr_t *addr, u16_t port);

/*Initialization client connection */
void udpClient_connect(void);

static void udpClient_send(void);

uint32_t Flash_Write_Data (uint32_t StartWriteSectorAddress, uint32_t *Data, uint16_t numberofwords);

void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords);

void Convert_To_Str (uint32_t *Data, char *Buf);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  udpClient_connect();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*handles the incoming data, it detertmines the type of packet
	  		 *received and calls the appropriate input function in low lavel ICMP(ping)*/

	  		ethernetif_input(&gnetif);/*Read a received packet from the Ethernet buffers*/
	  		/* Send it to the lwIP stack for handling .Worries about treatment for low level packets ICMP(ping)*/

	  		if (udp_recv_data)
	  		{
	  			udp_recv_data = 0;
	  			/*wtiting data sart from StartSectorAddress + 8 byte*/
	  			Flash_Write_Data(0x080C0000,(char *) rx_flash_buf, buf_len);  /*write data[] to FLASH_SECTOR_7*/
	  			Flash_Read_Data(0x080C0000,(char *) tx_flash_buffer, buf_len);/*read data from FLASH_SECTOR_7*/

	  			memcpy(data_send,tx_flash_buffer,buf_len);
				udpClient_send();
				memset(data_send,'\0',buf_len);


			}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void udpClient_send(void) {

	/*Pbuf is the packet buffer structure
	 *and it contains the information related
	 *to the packet sent. Information like the payload it is size length
	 *for this buffer .
	 *Also we have UDP control block, and it contains information about the
	 *client and the server.
	 *Information like the IP ADDRESS and PORT */


	/*we create a packet buffer that we are going to */
	struct pbuf *txBuf;
	/*Get the IP of the Client */
	/*The address witch is passed here(in function udp_receive_callback)
	 * is a 32-bit integer and wee nedd to converted it into proper
	 * address format */
	char data[200];

	/*Mixing the date send by the client with some additional data  */
	int len = sprintf(data, " UDP CLIENT SEND: ~ %s ~ \r\n",tx_flash_buffer);

	/* allocate memmory for this packet buffer*/
	txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);

	if (txBuf != NULL) {

		/* copy data to pbuf */
		/*Copy application supplied data into a pbuf.
		 * This function can only be used to copy the equivalent of buf->tot_len data.
		 * txBuf pbuf to fill with data
		 * data application supplied data buffer
		 * len length of the application supplied data buffer
		 * ERR_OK if successful, ERR_MEM if the pbuf is not big enough. */
		pbuf_take(txBuf, data, len);

		/* send udp data */
		/*Send data to a specified address using UDP.
		 * upcb UDP PCB used to send the data.
		 * txBuf chain of pbuf's to be sent.
		 * dest_adder Destination IP address.
		 * dst_port Destination UDP port. */
		udp_sendto(upcb, txBuf, &des_adder, des_port);

		/* free pbuf */
		pbuf_free(txBuf);
	}
}
/* this callback will called , when a client sends
 * some data to the server (OUR STM)                */
/* Process the datagram packet and send a reply to client. */

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
		const ip_addr_t *addr, u16_t port)
{
	/* global variable for buffer length*/
	buf_len = p->len;
	/*check if a data not bigger that buffer*/
	if (buf_len < 200)
	{
				memcpy(buffer, p->payload, p->len);
				des_adder = *addr;
				des_port = port;

				/*flag who do trigger for udp_receive_callback
				  function after get USER massage */
				udp_recv_data = 1;

				/*copy pbuf recieve_callback_buffer for .....*/
			memcpy(rx_flash_buf,buffer,buf_len);
	}

	/* Free receive pbuf */
	pbuf_free(p);

}
void udpClient_connect(void) {
	 /* Create a new UDP control block  */
	upcb = udp_new();

	 /* Bind the upcb to the local port -> IP_ANY_TYPE if we have
	 *  more then one Ethernet connection its for all */
    uint8_t	err = udp_bind(upcb, IP_ANY_TYPE, STM_PORT);/*we need to bind this block
	 	 	 	 	 	 	 	 	 	        to the ip address and a local port*/

    /*Wait until data packet arrives from client.*/
	/* Set a receive callback for the upcb */
	if(err == ERR_OK)
	   {
		/*Set up a receive callback which will be called
		 *  whenever a client send something*/
		   udp_recv(upcb,udp_receive_callback, NULL); /*udp_receive_callback function
		    											 is our receive callback. This callback
		    											 will be called when a client sends some DATA
		    											 to the server*/
	   }
	   else
	   {
		   /*pcb UDP PCB to be removed. The PCB is removed from the list of
		    * UDP PCB's and the data structure is freed from memory.*/
		   udp_remove(upcb);
	   }
}
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < 0x08007FFF) && (Address >= 0x08000000))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < 0x0800FFFF) && (Address >= 0x08008000))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < 0x08017FFF) && (Address >= 0x08010000))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < 0x0801FFFF) && (Address >= 0x08018000))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < 0x0803FFFF) && (Address >= 0x08020000))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < 0x0807FFFF) && (Address >= 0x08040000))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < 0x080BFFFF) && (Address >= 0x08080000))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < 0x080FFFFF) && (Address >= 0x080C0000))
  {
    sector = FLASH_SECTOR_7;
  }

  return sector;
}
uint32_t Flash_Write_Data (uint32_t StartWriteSectorAddress, uint32_t *Data, uint16_t numberofwords)
{
	//struct pbuf *p;
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	int counter_numberofwords = 0;

	uint32_t *temp_counter_numberofwords = 0;
	int sofar=0;


	 /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Erase the user Flash area */

	  /* Get the number of sector to erase from 1st sector */
	  StartWriteSectorAddress+=4; /*start write data from start address + 8 byte to store buffer length*/
	  uint32_t StartSector = GetSector(StartWriteSectorAddress);
	  uint32_t EndSectorAddress = StartWriteSectorAddress + numberofwords*4;
	  uint32_t EndSector = GetSector(EndSectorAddress);

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Sector        = StartSector;
	  EraseInitStruct.NbSectors     = (EndSector - StartSector) + 1;

	  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	     you have to make sure that these data are rewritten before they are accessed during code
	     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	     DCRST and ICRST bits in the FLASH_CR register. */
	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	  	  {
		  	  return HAL_FLASH_GetError ();
	  	  }
//	   counter_numberofwords = numberofwords;//32
	  HAL_FLASH_Lock();/*We lock flash for register CR to start read data*/

	  *temp_counter_numberofwords = *(__I uint32_t *)(StartWriteSectorAddress-4);
	  counter_numberofwords += *temp_counter_numberofwords;

	  HAL_FLASH_Unlock();/*we unlock flash for register CR to start write*/
	  /*Store in first address a number of words(buffer length) */
	   HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (StartWriteSectorAddress-4), (uint32_t)numberofwords);

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	  while (sofar<=numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartWriteSectorAddress, Data[sofar]) == HAL_OK)
	     {
	    	 StartWriteSectorAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();

	   return 0;
}


void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	StartSectorAddress+=4;
	while (1)
	{

		*RxBuf = *(__IO uint32_t *)StartSectorAddress;
		StartSectorAddress += 4;
		RxBuf++;
		if (!(numberofwords--)) break;
	}
}

void Convert_To_Str (uint32_t *Data, char *Buf)
{
	int numberofbytes = ((strlen((char *)Data)/4) + ((strlen((char *)Data) % 4) != 0)) *4;

	for (int i=0; i<numberofbytes; i++)
	{
		Buf[i] = Data[i/4]>>(8*(i%4));
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

