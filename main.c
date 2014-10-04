/**
  ******************************************************************************
  * @file    HRTIM/HRTIM_BasicPWM/Src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-June-2014
  * @brief   This example describes how to generate basic PWM waveforms with the 
  * HRTIM, as per HRTIM Cookbook basic examples.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F3xx_HAL_Examples
  * @{
  */

/** @addtogroup HRTIM_BasicPWM
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
#define HRTIM_INPUT_CLOCK       ((uint64_t)144000000)   /* Value in Hz */

/* Formula below works down to 70.3kHz (with presc ratio = 1) */ 
#define _1000KHz_PERIOD ((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 1000000))

/* Formula below works down to 70.3kHz (with presc ratio = 1) */ 
#define _200KHz_PERIOD ((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 200000))
#define _200Khz_DUTY_MAX (_200KHz_PERIOD/5)

/* Formula below works down to 70.3kHz (with presc ratio = 1) */ 
#define _100KHz_PERIOD ((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 100000))

/* Formula below works down to 70.3kHz (with presc ratio = 1) */ 
#define _100KHz_Plus_PERIOD ((uint16_t)((HRTIM_INPUT_CLOCK * 32) / 100001))

/* Formula below works down to 17.6kHz (with presc ratio = 4) */ 
#define _33KHz_PERIOD ((uint16_t)((HRTIM_INPUT_CLOCK * 8) / 33333))

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void Error_Handler_blink(uint32_t delay);

static void HRTIM_ClearHandle(HRTIM_HandleTypeDef *hhrtim);
static void GPIO_HRTIM_outputs_Config(void);

static void HRTIM_Config_MultiplePWM(void);
static void ADC_Config(void);
static uint16_t adc_read(uint32_t Channel, uint32_t Ts);
  
/* Private functions ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
HRTIM_HandleTypeDef hhrtim;
ADC_HandleTypeDef AdcHandle;
__IO uint32_t ADCReadout; //Define __IO as volatile if compiler gives error


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  
  /* STM32F3xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();


  /* Initialize STM32F3348-DISCO LEDs */
  BSP_LED_Init(LED2);   // Indicates MCU is active
  BSP_LED_Off(LED2);
  
  
  
    /* Configure the system clock */
    /* DO NOT set PLL again, something else does it already. */
  SystemClock_Config();
  
  HRTIM_Config_MultiplePWM();   /* Initialize HRTIM and related inputsm including ADC */
  GPIO_HRTIM_outputs_Config();  /* Initialize GPIO's HRTIM outputs */


  /* Init ADC, set triggers / channels */
  ADC_Config();

  uint16_t refint_cal      = (uint16_t) *(__IO uint32_t *)((uint32_t)0x1FFFF7BA);
  uint32_t calibration_multiplier = 0; // to calibrated millivolts
  ADCReadout = adc_read(ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_181CYCLES_5); //Update calibration
  calibration_multiplier = ADCReadout * 1000 / refint_cal; 
  ADCReadout *= calibration_multiplier;


    /* Normal operation */
  uint16_t duty = 0;
  uint16_t CHANNEL = 0;
  HRTIM_CompareCfgTypeDef compare_config;
  while (1)
  { 
    duty = 32;

    while(duty++ < _200Khz_DUTY_MAX)
    {
        /* Set compare registers for duty cycle on TA1 */
        compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
        compare_config.AutoDelayedTimeout = 0;
        compare_config.CompareValue = duty; 
        
        if(CHANNEL % 3 == 0){
        HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                      HRTIM_TIMERINDEX_TIMER_A,
                                      HRTIM_COMPAREUNIT_1,
                                      &compare_config);
        }
        if(CHANNEL % 3 == 1){
        HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                      HRTIM_TIMERINDEX_TIMER_B,
                                      HRTIM_COMPAREUNIT_1,
                                      &compare_config);
        }
            if(CHANNEL % 3 == 2){
        HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                      HRTIM_TIMERINDEX_TIMER_E,
                                      HRTIM_COMPAREUNIT_1,
                                      &compare_config);
        }
    HAL_Delay(1);
    }
    
    while(duty-- > 32)
    {
        /* Set compare registers for duty cycle on TA1 */
        compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
        compare_config.AutoDelayedTimeout = 0;
        compare_config.CompareValue = duty; 
        
        if(CHANNEL % 3 == 0){
        HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                      HRTIM_TIMERINDEX_TIMER_A,
                                      HRTIM_COMPAREUNIT_1,
                                      &compare_config);
        }
        if(CHANNEL % 3 == 1){
        HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                      HRTIM_TIMERINDEX_TIMER_B,
                                      HRTIM_COMPAREUNIT_1,
                                      &compare_config);
        }
            if(CHANNEL % 3 == 2){
        HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                      HRTIM_TIMERINDEX_TIMER_E,
                                      HRTIM_COMPAREUNIT_1,
                                      &compare_config);
        }
    HAL_Delay(1);
    }
    CHANNEL++;
    
    BSP_LED_Toggle(LED2);
  }
  
}


/**
* @brief  Configure GPIO outputs for the HRTIM
* @param  None
* @retval None
*/
static void GPIO_HRTIM_outputs_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable GPIOA clock for timer A outputs */
  __GPIOA_CLK_ENABLE();

  /* Configure HRTIM output: TA1 (PA8) and TA2 (PA9) and TB1 (PA10) */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
  
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Enable GPIOB clock for timer B outputs */
  __GPIOB_CLK_ENABLE();

  /* Configure HRTIM output: TD1 (PB14) and TD2 (PB15)*/
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15; 
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL; 
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;  
  GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;
  
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  
  /* Enable GPIOC clock for timer E outputs */
  __GPIOC_CLK_ENABLE();

  /* Configure HRTIM output: TE1 (PC8) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;  
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;  
  GPIO_InitStruct.Alternate = GPIO_AF3_HRTIM1; //<--- Not same AF as with TA and TB.
  
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
* @brief  HRTIM configuration for generating multiple PWM outputs
* @param  None
* @retval None
*/
static void HRTIM_Config_MultiplePWM(void)
{
  HRTIM_TimeBaseCfgTypeDef timebase_config;
  HRTIM_TimerCfgTypeDef timer_config;
  HRTIM_OutputCfgTypeDef output_config_TB1;
  HRTIM_OutputCfgTypeDef output_config_TA1;
  HRTIM_OutputCfgTypeDef output_config_TE1;
  HRTIM_CompareCfgTypeDef compare_config;
  HRTIM_ADCTriggerCfgTypeDef ADCTriggerConfig;



  /* ----------------------------------------------- */
  /* HRTIM Global initialization: Clock and DLL init */
  /* ----------------------------------------------- */
  /* Initialize the HRTIM handle (and clear it before) */
  HRTIM_ClearHandle(&hhrtim);
  hhrtim.Instance = HRTIM1;

 HAL_StatusTypeDef hrtimStatus;
  /* Initialize HRTIM */
  hrtimStatus = HAL_HRTIM_Init(&hhrtim);
  
  if(hrtimStatus != HAL_OK)
     Error_Handler_blink(150); /* if INIT went wrong */
     
  /* Select, enable clocks */
  __HAL_RCC_HRTIM1_CONFIG(RCC_HRTIM1CLK_PLLCLK);
  __HRTIM1_CLK_ENABLE();

  /* HRTIM DLL calibration: periodic calibration, set period to 14µs */
  hrtimStatus = HAL_HRTIM_DLLCalibrationStart(&hhrtim, HRTIM_CALIBRATIONRATE_14);
  
   if(hrtimStatus != HAL_OK)
     Error_Handler_blink(3000); /* if calibration went wrong */
     

     
  /* Wait calibration completion*/
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim, 1250) == HAL_TIMEOUT)
  {
    Error_Handler_blink(150); /* if DLL or clock is not correctly set */
  }
  
  /* ------------------------------------------------------------ */
  /* Timebase configuration: set PWM frequency and continuous mode */
  /* ------------------------------------------------------------ */
  timebase_config.Period = _200KHz_PERIOD;
  timebase_config.RepetitionCounter = 0;
  timebase_config.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  timebase_config.Mode = HRTIM_MODE_CONTINUOUS;

  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &timebase_config);
  
  timebase_config.Mode = HRTIM_MODE_SINGLESHOT; //Slave timers use single shot, triggered by MASTER
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, &timebase_config);
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, &timebase_config);
  HAL_HRTIM_TimeBaseConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, &timebase_config);

  
  /* --------------------------------------------------------- */
  /* TIMER global configuration: preload enabled on REP event */
  /* --------------------------------------------------------- */
  timer_config.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  timer_config.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  timer_config.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  timer_config.DACSynchro = HRTIM_DACSYNC_NONE;
  timer_config.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  timer_config.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  timer_config.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  timer_config.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
  timer_config.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  timer_config.InterruptRequests = HRTIM_TIM_IT_NONE;
  timer_config.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  timer_config.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  timer_config.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  timer_config.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  timer_config.DelayedProtectionMode = HRTIM_TIMDELAYEDPROTECTION_DISABLED;
  timer_config.UpdateTrigger= HRTIM_TIMUPDATETRIGGER_NONE;
  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_MASTER, &timer_config);
  
  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_A, &timer_config);
  
  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP1;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_B, &timer_config);
  
  timer_config.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_CMP2;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim, HRTIM_TIMERINDEX_TIMER_E, &timer_config);
    
 

  

  /* ------------------------------------------------------ */
  /* TA1 waveform description: set on period, reset on CMP1 */
  /* ------------------------------------------------------ */
  output_config_TA1.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  output_config_TA1.SetSource = HRTIM_OUTPUTSET_TIMPER;
  output_config_TA1.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1;
  output_config_TA1.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  output_config_TA1.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_config_TA1.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  output_config_TA1.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_config_TA1.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim,
                                 HRTIM_TIMERINDEX_TIMER_A,
                                 HRTIM_OUTPUT_TA1,
                                 &output_config_TA1);

  

  /* Set compare registers for duty cycle on TA1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = _200KHz_PERIOD/10;     /* 10% duty cycle */
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_A,
                                  HRTIM_COMPAREUNIT_1,
                                  &compare_config);

  /* Set compare registers for ADC trigger on TA1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (_200KHz_PERIOD/10) - 10;     // TODO 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_A,
                                  HRTIM_COMPAREUNIT_2,
                                  &compare_config);


                      
  /* ------------------------------------------------------ */
  /* TB1 waveform description: set on period, reset on CMP1 */
  /* ------------------------------------------------------ */
  output_config_TB1.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  output_config_TB1.SetSource = HRTIM_OUTPUTSET_TIMPER;
  output_config_TB1.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1;
  output_config_TB1.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  output_config_TB1.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_config_TB1.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  output_config_TB1.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_config_TB1.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim,
                                 HRTIM_TIMERINDEX_TIMER_B,
                                 HRTIM_OUTPUT_TB1,
                                 &output_config_TB1);

  /* Set compare registers for duty cycle on TB1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = _200KHz_PERIOD/10;     /* 10% duty cycle */
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_B,
                                  HRTIM_COMPAREUNIT_1,
                                  &compare_config);
                                  
  /* Set compare registers for ADC trigger on TB1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (_200KHz_PERIOD/10) - 10;     // TODO 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_B,
                                  HRTIM_COMPAREUNIT_2,
                                  &compare_config);
                                  

  /* ------------------------------------------------------ */
  /* TE1 waveform description: set on period, reset on CMP1 */
  /* ------------------------------------------------------ */
  output_config_TE1.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  output_config_TE1.SetSource = HRTIM_OUTPUTSET_TIMPER;
  output_config_TE1.ResetSource  = HRTIM_OUTPUTRESET_TIMCMP1;
  output_config_TE1.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  output_config_TE1.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  output_config_TE1.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  output_config_TE1.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  output_config_TE1.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim,
                                 HRTIM_TIMERINDEX_TIMER_E,
                                 HRTIM_OUTPUT_TE1,
                                 &output_config_TE1);

  /* Set compare registers for duty cycle on TE1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = _200KHz_PERIOD/10;     /* 10% duty cycle */
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_E,
                                  HRTIM_COMPAREUNIT_1,
                                  &compare_config);
                                  
  /* Set compare registers for ADC trigger on TE1 */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = (_200KHz_PERIOD/10) - 10;     // TODO 
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_TIMER_E,
                                  HRTIM_COMPAREUNIT_2,
                                  &compare_config);
                                  
                                  
  /* Set compare registers for trigger in Master timer */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = _200KHz_PERIOD/3;     /* Trigger at 1 / 3 of duty cycle */
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_MASTER,
                                  HRTIM_COMPAREUNIT_1,
                                  &compare_config);  
                                  
  /* Set compare registers for trigger in Master timer */
  compare_config.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  compare_config.AutoDelayedTimeout = 0;
  compare_config.CompareValue = 2*_200KHz_PERIOD/3;     /* Trigger at 2 / 3 of duty cycle */
  HAL_HRTIM_WaveformCompareConfig(&hhrtim,
                                  HRTIM_TIMERINDEX_MASTER,
                                  HRTIM_COMPAREUNIT_2,
                                  &compare_config);      
                                  
  /* Configure triggers for ADC. */
  ADCTriggerConfig.Trigger      = HRTIM_ADCTRIGGEREVENT13_TIMERA_CMP2 | HRTIM_ADCTRIGGEREVENT13_TIMERB_CMP2 | HRTIM_ADCTRIGGEREVENT13_TIMERE_CMP2;
  ADCTriggerConfig.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A | HRTIM_ADCTRIGGERUPDATE_TIMER_B | HRTIM_ADCTRIGGERUPDATE_TIMER_E;
  if(HAL_HRTIM_ADCTriggerConfig(&hhrtim, HRTIM_ADCTRIGGER_1, &ADCTriggerConfig) != HAL_OK)
  {
  Error_Handler();
  }                            
                                  

  /* ---------------*/
  /* HRTIM start-up */
  /* ---------------*/
  /* Enable HRTIM's outputs TA1, TB1 and TE1 */
  /* Note: it is necessary to enable also GPIOs to have outputs functional */
  /* This must be done after HRTIM initialization */
  HAL_HRTIM_WaveformOutputStart(&hhrtim, HRTIM_OUTPUT_TA1
                                       + HRTIM_OUTPUT_TB1
                                       + HRTIM_OUTPUT_TE1);

  /* Start HRTIM's TIMER A & B & E */
  HAL_HRTIM_WaveformCounterStart(&hhrtim, HRTIM_TIMERID_MASTER + HRTIM_TIMERID_TIMER_A + HRTIM_TIMERID_TIMER_B + HRTIM_TIMERID_TIMER_E);

}

/**
* @brief  HRTIM_ClearHandle: make sure the handle is correctly initialized even 
*                            if placed in a non zero-init RAM region
* @param  None
* @retval None
*/
static void HRTIM_ClearHandle(HRTIM_HandleTypeDef *hhrtim)
{
  hhrtim->Instance = (HRTIM_TypeDef *)NULL;
  
  hhrtim->hdmaMaster = (DMA_HandleTypeDef *)NULL;    
  hhrtim->hdmaTimerA = (DMA_HandleTypeDef *)NULL;     
  hhrtim->hdmaTimerB = (DMA_HandleTypeDef *)NULL;  
  hhrtim->hdmaTimerC = (DMA_HandleTypeDef *)NULL;  
  hhrtim->hdmaTimerD = (DMA_HandleTypeDef *)NULL;  
  hhrtim->hdmaTimerE = (DMA_HandleTypeDef *)NULL;  
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = RCC_PLL_MUL16 (16)
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  if(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
  {  
  /* Enable HSI oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSEState            = RCC_HSE_OFF;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16; // 64 MHz (8 MHz/2 * 16)    
      
      HAL_StatusTypeDef oscStatus = HAL_RCC_OscConfig(&RCC_OscInitStruct);
      if (oscStatus == HAL_TIMEOUT)
      {
        Error_Handler_blink(500);
      }
      if (oscStatus == HAL_ERROR)
      {
        Error_Handler_blink(2500);
      }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    Error_Handler_blink(1000);
  }
  
  /* Setup ADC clock */
  __ADC12_CLK_ENABLE();  
  __HAL_RCC_ADC12_CONFIG(RCC_ADC12PLLCLK_DIV1);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 (GREEN) off */
  volatile uint32_t counter = 1;
  while(1)
  {
      HAL_Delay(100);
        BSP_LED_Off(LED2);
      HAL_Delay(100);
        BSP_LED_On(LED2);
      
  }
}

static void Error_Handler_blink(uint32_t delay)
{
  while(1)
  {
      HAL_Delay(delay);
        BSP_LED_Off(LED2);
      HAL_Delay(delay);
        BSP_LED_On(LED2);
      
  }
}

/**
 * Configure ADC, including triggers
 */
static void ADC_Config(void)
{
  GPIO_InitTypeDef           GPIO_InitStructure;
  ADC_ChannelConfTypeDef     ChannelConfig;
        
  // Configure ADC
        AdcHandle.Instance                   = ADC2; 
        AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV1;
        AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
        AdcHandle.Init.ScanConvMode          = ENABLE;
        AdcHandle.Init.ContinuousConvMode    = DISABLE;
        AdcHandle.Init.DiscontinuousConvMode = ENABLE;
        AdcHandle.Init.NbrOfDiscConversion   = 1;
        AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
        AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONVHRTIM_TRG1;
        AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
        AdcHandle.Init.NbrOfConversion       = 3;
        AdcHandle.Init.DMAContinuousRequests = DISABLE;
        AdcHandle.Init.EOCSelection          = DISABLE;
        
        if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
        {
            Error_Handler_blink(100);
        }
        
      //Configure pins
      /* Configure ADC2 Channel1 as analog input / PA4  */
      GPIO_InitStructure.Pin  =  GPIO_PIN_4;
      GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStructure.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

      /* Configure ADC2 Channel4 as analog input / PA7  */
      GPIO_InitStructure.Pin  =  GPIO_PIN_7;
      GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStructure.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
      
      ChannelConfig.Channel      = ADC_CHANNEL_4;
      ChannelConfig.Offset       = 0;
      ChannelConfig.OffsetNumber = ADC_OFFSET_NONE;
      ChannelConfig.Rank         = ADC_REGULAR_RANK_1;
      ChannelConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
      ChannelConfig.SingleDiff   = ADC_SINGLE_ENDED;

      if(HAL_ADC_ConfigChannel(&AdcHandle, &ChannelConfig) != HAL_OK)
      {
        Error_Handler();
      }
      
      /* Configure ADC2 Channel15 as analog input / PB15  */
      GPIO_InitStructure.Pin  =  GPIO_PIN_15;
      GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStructure.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
      
      ChannelConfig.Channel      = ADC_CHANNEL_15;
      ChannelConfig.Offset       = 0;
      ChannelConfig.OffsetNumber = ADC_OFFSET_NONE;
      ChannelConfig.Rank         = ADC_REGULAR_RANK_2;
      ChannelConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
      ChannelConfig.SingleDiff   = ADC_SINGLE_ENDED;

      if(HAL_ADC_ConfigChannel(&AdcHandle, &ChannelConfig) != HAL_OK)
      {
        Error_Handler();
      }
      
      /* Configure ADC2 Channel11 as analog input / PB0  */
      GPIO_InitStructure.Pin  =  GPIO_PIN_10;
      GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStructure.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
      
      ChannelConfig.Channel      = ADC_CHANNEL_11;
      ChannelConfig.Offset       = 0;
      ChannelConfig.OffsetNumber = ADC_OFFSET_NONE;
      ChannelConfig.Rank         = ADC_REGULAR_RANK_3;
      ChannelConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;
      ChannelConfig.SingleDiff   = ADC_SINGLE_ENDED;

      if(HAL_ADC_ConfigChannel(&AdcHandle, &ChannelConfig) != HAL_OK)
      {
        Error_Handler();
      }
      
}

static uint16_t adc_read(uint32_t Channel, uint32_t Ts){
    ADC_ChannelConfTypeDef sConfig;

    AdcHandle.Instance = ADC2; 

    // Configure ADC channel
    sConfig.Rank         = 4;
    sConfig.SamplingTime = Ts;
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    sConfig.Channel      = Channel;

   if ( HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
    {
            Error_Handler_blink(150);
    }

    if( HAL_ADC_Start(&AdcHandle)!= HAL_OK)
    {
            Error_Handler_blink(3000);
    }
    // Wait end of conversion and get value
    if (HAL_ADC_PollForConversion(&AdcHandle, 200) == HAL_OK) {
        return (HAL_ADC_GetValue(&AdcHandle));
    } else {
        return 5000;
    }
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

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

