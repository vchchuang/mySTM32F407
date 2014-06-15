/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_hid_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

//Library config for this project!!!!!!!!!!!
#include "stm32f4xx_conf.h"

/** @addtogroup STM32F4-Discovery_Demo
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define TESTRESULT_ADDRESS         0x080FFFFC
#define ALLTEST_PASS               0x00000000
#define ALLTEST_FAIL               0x55555555

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;
 
 
 #define LED_PORT GPIOB

 
uint16_t PrescalerValue = 0;

__IO uint32_t TimingDelay;
__IO uint8_t DemoEnterCondition = 0x00;
__IO uint8_t UserButtonPressed = 0x00;
LIS302DL_InitTypeDef  LIS302DL_InitStruct;
LIS302DL_FilterConfigTypeDef LIS302DL_FilterStruct;  
__IO int8_t X_Offset, Y_Offset, Z_Offset  = 0x00;
uint8_t Buffer[6];
 GPIO_InitTypeDef GPIO_InitStructure , GPIO_InitStructure2,GPIO_InitStructure3;
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct,TIM_TimeBaseInitStruct2;
  TIM_OCInitTypeDef TIM_OCInitStruct,TIM_OCInitStruct2;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
 
 int brightness = 1000-1;
  
/* Private function prototypes -----------------------------------------------*/
static uint32_t Demo_USBConfig(void);
static void TIM3_Config(void);
static void Demo_Exec(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  
  while(TimingDelay--);
  
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);
   
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC  | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA, ENABLE );
   
   RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM3, ENABLE );

 
 
    volatile int i;
    int n = 1;
    
     
    GPIO_StructInit(&GPIO_InitStructure); // Reset init structure
    GPIO_StructInit(&GPIO_InitStructure2); // Reset init structure
    GPIO_StructInit(&GPIO_InitStructure3); // Reset init structure
 
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
    
    //GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    //GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

/**
	PWM
		Output	PortC 6 7 8 9
		AF TIM3				
								**/    
    

    // Setup Servo on STM32-Discovery Board to use PWM.
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            // Alt Function - Push Pull
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( GPIOC, &GPIO_InitStructure );  
    

/**
	Button
		Output PortB 4 5
						 **/

    /* GPIOA Configuration: CH1 (PB4) and CH2 (PB5) */
    GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 ;
    GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure2);

    GPIOB->BSRRL = GPIO_Pin_4;
    GPIOB->BSRRL = GPIO_Pin_5;

/**
	Button
		Input PortA 2 3
						**/

    /* GPIOA Configuration: (PA2) and (PA3) */     //input
    GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 ;
    GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure2.GPIO_PuPd = GPIO_PuPd_DOWN ;
    GPIO_Init(GPIOA, &GPIO_InitStructure2); 
    
/**	
	Button A2 A3 Interrupt
	EXTI 2 3		
							**/ 

    //清空中断标志
    EXTI_ClearITPendingBit(EXTI_Line2);
    EXTI_ClearITPendingBit(EXTI_Line3);

    //选择中断管脚PA.2 PA.3 
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource2);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
  //    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
  //   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
    
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line2  ; //选择中断线路2 
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //设置为中断请求，非事件请求
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //设置中断触发方式为上下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;            //外部中断使能
    EXTI_Init(&EXTI_InitStructure);
    EXTI_GenerateSWInterrupt(EXTI_Line2);
    
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line3 ; //选择中断线路2 
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; //设置为中断请求，非事件请求
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //设置中断触发方式为上下降沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;            //外部中断使能
    EXTI_Init(&EXTI_InitStructure);
    EXTI_GenerateSWInterrupt(EXTI_Line3);
   
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);      //选择中断分组2
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn  ;     //选择中断通道2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式中断优先级设置为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级设置为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                   //使能中断
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);    //选择中断分组2
    NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;     //选择中断通道2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占式中断优先级设置为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //响应式中断优先级设置为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                   //使能中断
    NVIC_Init(&NVIC_InitStructure);
    
    
/**
	Timer setting
	Tim3
					**/
    
    PrescalerValue = (uint16_t) ((SystemCoreClock /4) / 100000) - 1;
    
    // Let PWM frequency equal 100Hz.
    // Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
    // Solving for prescaler gives 240.
    TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = 3360 - 1;   // 0..2000
    TIM_TimeBaseInitStruct.TIM_Prescaler = 500; 
    TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );
 /**
		TIM_Prescaler = 500-1, TIM_Period = 1680-1 , TIMxCLK = 84MHz
		輸出脈波週期 = (TIM_Period + 1) * (TIM_Prescaler + 1) / TIMxCLK
		= ( 1680 - 1 +1 ) * ( 500 - 1 + 1 ) / 84000000 = 0.01s(100Hz)
																		**/   

    TIM_OCStructInit( &TIM_OCInitStruct );
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    // Initial duty cycle equals 0%. Value can range from zero to 1000.
    TIM_OCInitStruct.TIM_Pulse = 3360 -1; // 0 .. 1680 (0=Always Off, 1680=Always On)
    
    
    TIM_OC1Init( TIM3, &TIM_OCInitStruct ); // Channel 1  Servo
    TIM_OC2Init( TIM3, &TIM_OCInitStruct ); // Channel 2  Servo
    TIM_OC3Init( TIM3, &TIM_OCInitStruct ); // Channel 3 PB7 Servo L
    TIM_OC4Init( TIM3, &TIM_OCInitStruct ); // Channel 4 PB8 Servo R
    TIM_Cmd( TIM3, ENABLE );

/** 
	Project Testing
						**/

  while(1)  // Do not exit
  {

//Test 1

	if(TIM3->CCR4>0){
		for(i=0;i<16000000;i++);
		TIM3->CCR4 = 0;
	}
/*
//Test 2
	TIM3->CCR4 = 168;
	for(i=0;i<16000000;i++);
	TIM3->CCR4 = 336;
	for(i=0;i<16000000;i++);
	TIM3->CCR4 = 252;
	for(i=0;i<16000000;i++);

/*
//Test 3

    if (((brightness + n) >= 3000) || ((brightness + n) <= 0))
      n = -n; // if  brightness maximum/maximum change direction
 
        brightness += n;
 
   	TIM3->CCR1 = brightness; // set brightness
    TIM3->CCR2 = 1000 - brightness; // set brightness
    TIM3->CCR3 = 3000 - brightness; // set brightness
    TIM3->CCR4 = 3000 - brightness; // set brightness
 
    for(i=0;i<10000;i++);  // delay
    //Delay(250000);
    //Delay(1000000);
    //Delay(1000000);
*/  
   }
 
  return(0); // System will implode
/**
	Others
			**/
  
  if (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)
  {
    if ((*(__IO uint32_t*) TESTRESULT_ADDRESS) == ALLTEST_PASS)
    {
     
      /* Waiting User Button is pressed or Test Program condition verified */
      while ((STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)&&(TimingDelay != 0x00))
      {}
    }
    else
    {
      /* Waiting User Button is Released  or TimeOut*/
     
      while ((STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)&&(TimingDelay != 0x00))
      {}
      if (STM_EVAL_PBGetState(BUTTON_USER) == Bit_RESET)
      {
        
      }
    }
    if (TimingDelay == 0x00)
    {
      /* Turn off LEDs available on STM32F4-Discovery ------------------------*/
      /* Waiting User Button is released */
      while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)
      {}
      
      /* Unlocks the FLASH control register access */
      FLASH_Unlock();
      
      /* Move discovery kit to detect negative and positive acceleration values 
      on X, Y and Z axis */
      Accelerometer_MEMS_Test();
      
      /* USB Hardware connection */
      USB_Test();
      
      /* Audio Hardware connection */
      Audio_Test();
      
      /* Microphone MEMS Hardware connection */
      Microphone_MEMS_Test();
      
      /* Write PASS code at last word in the flash memory */
      FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_PASS);
      
      while(1)
      {
        /* Toggle Green LED: signaling the End of the Test program */
        STM_EVAL_LEDToggle(LED4);
        Delay(41999/2);
      }
    }
    else
    {
      Demo_Exec();
    }
  }
  else
  {    
    Demo_Exec();
  }
}
/**
	Button L
			**/
void EXTI2_IRQHandler(void)
{
 if(EXTI_GetITStatus(EXTI_Line2) != 0)
 {
    EXTI_ClearITPendingBit(EXTI_Line2);   
    

    TIM3->CCR3 = 3000;

    
    EXTI_ClearITPendingBit(EXTI_Line2);
    EXTI_ClearITPendingBit(EXTI_Line3);
  }
}
/**
	Button R
				**/
void EXTI3_IRQHandler(void)
{
/*
int i,n;
 while(1)  // Do not exit
  {

    if (((brightness + n) >= 3000) || ((brightness + n) <= 0))
      n = -n; // if  brightness maximum/maximum change direction
 
        brightness += n;
 
    TIM3->CCR4 = 3000 - brightness; // set brightness
 
    for(i=0;i<1000;i++);  // delay
    //Delay(250000);
    //Delay(1000000);
    //Delay(1000000);
  
   }
    
    EXTI_ClearITPendingBit(EXTI_Line2);
    EXTI_ClearITPendingBit(EXTI_Line3);
 */
int l=0;
 if(EXTI_GetITStatus(EXTI_Line3) != 0)
 {
   EXTI_ClearITPendingBit(EXTI_Line3);
    /*
		SG90 
			 0 				  90				180
				ARR>3360 20ms  	AAR>3360 20ms		ARR>3360 20ms
				CCR>252 1.5ms  	CCR>168	 1.0ms		CCR>336 2.0ms
																	*/
	TIM3->CCR4 = 336; //
	for(l=0;l<16000000;l++);
	TIM3->CCR4 = 252;	//
	for(l=0;l<16000000;l++);
	TIM3->CCR4 = 168;	//
	for(l=0;l<16000000;l++);
	//for(l=0;l<10000;l++);
    //TIM3->CCR4 = 2240;
	//for(l=0;l<10000;l++);

    EXTI_ClearITPendingBit(EXTI_Line2);
    EXTI_ClearITPendingBit(EXTI_Line3);
    }

}


/**
  * @brief  Execute the demo application.
  * @param  None
  * @retval None
  */
static void Demo_Exec(void)
{
  RCC_ClocksTypeDef RCC_Clocks;
  uint8_t togglecounter = 0x00;
  
  while(1)
  {
    DemoEnterCondition = 0x00;
    
    /* Reset UserButton_Pressed variable */
    UserButtonPressed = 0x00;
    
    /* SysTick end of count event each 10ms */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);  
    

    
    /* Waiting User Button is pressed */
    while (UserButtonPressed == 0x00)
    {
      
    }
    
    /* Waiting User Button is Released */
    while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)
    {}
    UserButtonPressed = 0x00;
    

    /* MEMS configuration */
    LIS302DL_InitStruct.Power_Mode = LIS302DL_LOWPOWERMODE_ACTIVE;
    LIS302DL_InitStruct.Output_DataRate = LIS302DL_DATARATE_100;
    LIS302DL_InitStruct.Axes_Enable = LIS302DL_XYZ_ENABLE;
    LIS302DL_InitStruct.Full_Scale = LIS302DL_FULLSCALE_2_3;
    LIS302DL_InitStruct.Self_Test = LIS302DL_SELFTEST_NORMAL;
    LIS302DL_Init(&LIS302DL_InitStruct);
    
    /* Required delay for the MEMS Accelerometre: Turn-on time = 3/Output data Rate 
    = 3/100 = 30ms */
    Delay(30);
    
    DemoEnterCondition = 0x01;
    /* MEMS High Pass Filter configuration */
    LIS302DL_FilterStruct.HighPassFilter_Data_Selection = LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER;
    LIS302DL_FilterStruct.HighPassFilter_CutOff_Frequency = LIS302DL_HIGHPASSFILTER_LEVEL_1;
    LIS302DL_FilterStruct.HighPassFilter_Interrupt = LIS302DL_HIGHPASSFILTERINTERRUPT_1_2;
    LIS302DL_FilterConfig(&LIS302DL_FilterStruct);
    
    LIS302DL_Read(Buffer, LIS302DL_OUT_X_ADDR, 6);
    X_Offset = Buffer[0];
    Y_Offset = Buffer[2];
    Z_Offset = Buffer[4];
    
    /* USB configuration */
    Demo_USBConfig();
    
    /* Waiting User Button is pressed */
    while (UserButtonPressed == 0x00)
    {}
    
    /* Waiting User Button is Released */
    while (STM_EVAL_PBGetState(BUTTON_USER) == Bit_SET)
    {}
    
    /* Disable SPI1 used to drive the MEMS accelerometre */
    SPI_Cmd(LIS302DL_SPI, DISABLE);
    
    /* Disconnect the USB device */
    DCD_DevDisconnect(&USB_OTG_dev);
    USB_OTG_StopDevice(&USB_OTG_dev);
  }
}

/**
  * @brief  Initializes the USB for the demonstration application.
  * @param  None
  * @retval None
  */
static uint32_t Demo_USBConfig(void)
{
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_HID_cb, 
            &USR_cb);
  
  return 0;
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
  * @brief  This function handles the test program fail.
  * @param  None
  * @retval None
  */
void Fail_Handler(void)
{
  /* Erase last sector */ 
  FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
  /* Write FAIL code at last word in the flash memory */
  FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_FAIL);
  
  while(1)
  {
    /* Toggle Red LED */
    STM_EVAL_LEDToggle(LED5);
    Delay(5);
  }
}

/**
  * @brief  MEMS accelerometre management of the timeout situation.
  * @param  None.
  * @retval None.
  */
uint32_t LIS302DL_TIMEOUT_UserCallback(void)
{
  /* MEMS Accelerometer Timeout error occured during Test program execution */
  if (DemoEnterCondition == 0x00)
  {
    /* Timeout error occured for SPI TXE/RXNE flags waiting loops.*/
    Fail_Handler();    
  }
  /* MEMS Accelerometer Timeout error occured during Demo execution */
  else
  {
    while (1)
    {   
    }
  }
  return 0;  
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
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


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

