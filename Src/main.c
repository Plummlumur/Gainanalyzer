
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "tim.h"
#include "gpio.h"
#include "fsmc.h"
#include "splash.h"
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include "MA_ILI9341.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BGCOLOR 			COLOR_WHITE
#define FCOLOR 				COLOR_BLACK
#define DEBOUNCE_TIME 		10
#define SPS 				196000
#define ADC_PRESCALER		1
#define ADC_SAMPLES 		ADC_PRESCALER*4096
#define VDD 				2.8
#define ADC_BITS			4096.0
#define MAX_AMP				20.0 //maximum Vpp
//#define MULTIPLIER			(20/VDD)*1.08 //to calculate the value between 0 and 20 Vpp
#define MULTIPLIER			(20.0/VDD)*1.076 //to calculate the value between 0 and 20 Vpp
#define BITDIVIDER			ADC_BITS/VDD //to calculate the "real" value between 0 and 3 V
#define STEP				100 //frequency steps for sweep
#define AVG					100	//for averaging the results
uint32_t pushedCtr;
struct  frame	//holds the UI information (the actual screen)
{
	uint8_t start;
	uint8_t frequency;
	uint8_t sweep;
	uint8_t amplify;
	uint8_t measure;
};

typedef struct frame UI;
UI ui = {1,0,0,0,0}; //start with ui.start =1
//uint32_t uptime = 0;
uint8_t longPress = 0;
uint8_t shortPress = 0;

int fDigit[5] = {0}; //holds frequency
int fcDigit = 0; //selected digit
int aDigit[4] = {0}; //holds  amplitude
int acDigit = 0; //selected digit
int frequency = 0; //frequency to be generated
uint32_t samples = 0; //numbers of samples per period
uint16_t *SineDynamic; //holds the dds table
uint16_t ADC_Measurement[ADC_SAMPLES]={0}; //to store the measurements
uint16_t amplitude = 0; //amplitude to be reached
float writeOut[2000][4];
float ADC_Result[ADC_SAMPLES] = {0};
volatile uint32_t debounceTime = 0;
UINT testByte;
float sum = 0; //holds the sum of ADC_Result
float rms0, rms1 = 0.0; //the rms value
uint8_t digit1 = 0;
char c[6];
uint32_t wert0, wert1;
float amp0 = 0.0, amp1 = 0.0, amp = 0;
float rmsMean1 = 0;
float rmsMean0 = 0;
float dc0 = 0, dc1=0;
uint8_t ctrl = 0;
uint8_t potOneValue[2];
uint8_t potTwoValue[2];
uint16_t aCache = 0;
uint8_t dc = 0;
uint8_t swp = 0;
FATFS myFS;
FIL myFILE;
FRESULT fr;
UINT testByte;

/*next 5 lines for digital filter, not implemented yet
float a0 = pow((1-exp(2*PI*0.1)),4);
float b1 = 4*exp(2*PI*0.1);
float b2 = -6*exp(2*PI*0.1);
float b3 = 4*pow(exp(2*PI*0.1),3);
float b4 = -pow(exp(2*PI*0.1),4);
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void clear_ui(UI *ui);
void sweep_ui(void);
void amplify_ui(void);
void frequency_ui(void);
void start_ui(void);
void measure_ui(void);
void call_ui(void);
int debounce(void);
void create_sine(uint16_t *sine, uint32_t samples, int frequency);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_ADC1_Init();
	MX_DAC_Init();
	MX_FSMC_Init();
	MX_I2C1_Init();
	MX_SDIO_SD_Init();
	MX_FATFS_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM8_Init();
	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
	ILI9341_Init();

	ILI9341_setRotation(2);
	ILI9341_Fill(BGCOLOR);
	potOneValue[0] = 0xA9;
	potOneValue[1] = 254;
	potTwoValue[0] = 0xAA;
	potTwoValue[1] = 254;
	HAL_I2C_Master_Transmit(&hi2c1, 0x50, potOneValue, 2,10 );
	HAL_I2C_Master_Transmit(&hi2c1, 0x50, potTwoValue, 2,10 );
	__HAL_TIM_CLEAR_IT(&htim1,TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim2,TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim8,TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim8);

	call_ui();
	if(f_mount(&myFS,SDPath, 1) == FR_OK)
	{


		char myPath[] = "FREQRESP.TXT\0";

		if(f_open(&myFILE, myPath, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK)
		{
			char myData[] = "Amplitude in, Amplitude out, Frequency\0, 20*log10(Out/In)";
			f_write(&myFILE, myData, sizeof(myData), &testByte);
		}
		f_close(&myFILE);


	}



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{//ToDo: Make averaging a user driven parameter
	rms0 = 0, rms1 = 0;
	if(ctrl)
	{
		float offSet0 = 0.0, offSet1 = 0.0;
		for(int i = 100; i< 100+2*samples; i+=2) // mean value (DC) as float
		{
			ADC_Result[i] = (float)ADC_Measurement[i];  // copy ADC data into a float array.
			ADC_Result[i+1] = (float)ADC_Measurement[i+1];
			offSet1 += ADC_Result[i];
			offSet0 += ADC_Result[i+1];
		}

		offSet1/=samples;
		offSet0/=samples;
		for(int i = 100; i< 100+2*samples; i+=2)
		{
			ADC_Result[i]-=offSet1;
			ADC_Result[i+1]-=offSet0;
		}
		for(int i = 100; i< 100+2*samples; i+=2)
		{
			ADC_Result[i]/=BITDIVIDER;
			ADC_Result[i+1]/=BITDIVIDER;
		}
		for(int i = 100; i< 100+2*samples; i+=2)
		{
			rms1 += pow(ADC_Result[i],2);
			rms0 += pow(ADC_Result[i+1],2);
		}
		rms1=sqrt(rms1/samples)*ADC_PRESCALER;
		rms0=sqrt(rms0/samples)*ADC_PRESCALER;
		rms1 *= MULTIPLIER;
		rms0 *= MULTIPLIER;
		rms0 *= 1/(8.5e-11*pow(frequency,2)-2.7e-6*frequency+1);
		amp1 = 2*rms1*sqrt(2);
		amp0 = 2*rms0*sqrt(2);
		if(amp<1)
		{
			potOneValue[1]=254;
			potTwoValue[1]=25;
			HAL_I2C_Master_Transmit(&hi2c1, 0x50, potOneValue, 2,10 );
			HAL_I2C_Master_Transmit(&hi2c1, 0x50, potTwoValue, 2,10 );
			ctrl = 0;
			clear_ui(&ui);
			ui.frequency = 1;
			call_ui();
		}
		if(amp1 > amp*1.01)
		{
			potOneValue[1]++;
			HAL_I2C_Master_Transmit(&hi2c1, 0x50, potOneValue, 2,10 );
			if(potOneValue[1] > 254)
				potOneValue[1] = 1;
		}
		else if(amp1 < amp*0.99)
		{
			potOneValue[1]--;
			HAL_I2C_Master_Transmit(&hi2c1, 0x50, potOneValue, 2,10 );
			if(potOneValue[1] < 1)
				potOneValue[1] = 254;
		}
		else
		{
			ctrl = 0;
			clear_ui(&ui);
			if(swp)
			{
				ui.sweep = 1;
			}
			else
			{
				ui.frequency = 1;
			}
			call_ui();
		}


	}
	if(ui.measure)
	{
		static uint8_t ctr = 0;
		float offSet0 = 0.0, offSet1 = 0.0;
		for(int i = 0; i< 2*samples; i+=2) // mean value (DC) as float
		{
			ADC_Result[i] = (float)ADC_Measurement[i];  // copy ADC data into a float array.
			ADC_Result[i+1] = (float)ADC_Measurement[i+1];
			offSet1 += ADC_Result[i];
			offSet0 += ADC_Result[i+1];
		}
		offSet1/=samples;
		offSet0/=samples;
		for(int i = 0; i< 2*samples; i+=2)
		{
			ADC_Result[i]-=offSet1;
			ADC_Result[i+1]-=offSet0;
		}
		for(int i = 0; i< 2*samples; i+=2)
		{
			ADC_Result[i]/=BITDIVIDER;
			ADC_Result[i+1]/=BITDIVIDER;
		}
		for(int i = 0; i< 2*samples; i+=2)
		{
			rms1 += pow(ADC_Result[i],2);
			rms0 += pow(ADC_Result[i+1],2);
		}


		rms1=sqrt(rms1/samples)*ADC_PRESCALER;
		rms0=sqrt(rms0/samples)*ADC_PRESCALER;
		rms0 *= 1/(8.5e-11*pow(frequency,2)-2.7e-6*frequency+1);
		amp1 = rms1*2*sqrt(2);
		amp0 = rms0*2*sqrt(2);
		rmsMean1 += rms1;
		rmsMean0 += rms0;
		if(ctr == AVG-1)
		{
			rmsMean1 /= AVG;
			rmsMean0 /= AVG;
			rmsMean1 *= MULTIPLIER;
			rmsMean0 *= MULTIPLIER;
			amp1 = 2*rmsMean1*sqrt(2);
			amp0 = 2*rmsMean0*sqrt(2);
			ctr = 0;
			dc1 = offSet1;
			dc0 = offSet0;
			dc1 /= BITDIVIDER;
			dc0 /= BITDIVIDER;
			dc0 += 0.07;
			dc1 += 0.07;
			if(dc)
			{
				sprintf(c, "%.2f", dc1);
				ILI9341_printText(c, 100,100, FCOLOR, BGCOLOR, 2);
				sprintf(c, "%.2f", dc0);
				ILI9341_printText(c, 100,160, FCOLOR, BGCOLOR, 2);

			}
			else
			{
				sprintf(c, "%.2f",rmsMean1);
				ILI9341_printText(c, 100,100, FCOLOR, BGCOLOR, 2);
				sprintf(c, "%.2f", amp1);
				ILI9341_printText(c, 100,120, FCOLOR, BGCOLOR, 2);
				sprintf(c, "%.2f", rmsMean0);
				ILI9341_printText(c, 100,160, FCOLOR, BGCOLOR, 2);
				sprintf(c, "%.2f", amp0);
				ILI9341_printText(c, 100,180, FCOLOR, BGCOLOR, 2);
			}


			rmsMean1 = 0;
			rmsMean0 = 0;
		}
		ctr++;
	}
	if(ui.sweep && swp)
	{

		static uint8_t ctr = 0;
		static uint16_t sweepCtr = 0;
		float offSet0 = 0.0, offSet1 = 0.0;
		for(int i = 100; i< 100+2*samples; i+=2) // mean value (DC) as float
		{
			ADC_Result[i] = (float)ADC_Measurement[i];  // copy ADC data into a float array.
			ADC_Result[i+1] = (float)ADC_Measurement[i+1];
			offSet1 += ADC_Result[i];
			offSet0 += ADC_Result[i+1];
		}
		offSet1/=samples;
		offSet0/=samples;
		for(int i = 100; i< 100+2*samples; i+=2)
		{
			ADC_Result[i]-=offSet1;
			ADC_Result[i+1]-=offSet0;
		}
		for(int i = 100; i< 100+2*samples; i+=2)
		{
			ADC_Result[i]/=BITDIVIDER;
			ADC_Result[i+1]/=BITDIVIDER;
		}
		for(int i = 100; i< 100+2*samples; i+=2)
		{
			rms1 += pow(ADC_Result[i],2);
			rms0 += pow(ADC_Result[i+1],2);
		}


		rms1=sqrt(rms1/samples)*ADC_PRESCALER;
		rms0=sqrt(rms0/samples)*ADC_PRESCALER;
		rms0 *= 1/(8.5e-11*pow(frequency,2)-2.7e-6*frequency+1);
		amp1 = rms1*2*sqrt(2);
		amp0 = rms0*2*sqrt(2);
		rmsMean1 += rms1;
		rmsMean0 += rms0;
		if(ctr >= AVG - 1)
		{
			/*calculate mean*/
			rmsMean1 /= AVG;
			rmsMean0 /= AVG;
			rmsMean1 *= MULTIPLIER;
			rmsMean0 *= MULTIPLIER;
			amp1 = 2*rmsMean1*sqrt(2);
			amp0 = 2*rmsMean0*sqrt(2);
			//ToDo:if amp is < or > a given threshold, then repeat the measurement at that frequency

				/*Write values*/
				writeOut[sweepCtr][0] = amp1;
				writeOut[sweepCtr][1] = amp0;
				writeOut[sweepCtr][2] = (float)frequency;
				writeOut[sweepCtr][3] = 20*log10(amp0/amp1);
				sweepCtr++;
				ctr = 0;
				rmsMean1 = 0;
				rmsMean0 = 0;
				/*Create and start next sine with new frequency */
				frequency += STEP;
				samples = SPS / frequency;
				TIM8->ARR = (HAL_RCC_GetHCLKFreq()/((TIM8->PSC+1)*samples*frequency));
				free(SineDynamic);
				SineDynamic = (uint16_t*) malloc(samples * sizeof(uint16_t));
				create_sine(&SineDynamic[0], samples, frequency);
				HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
				HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t * ) SineDynamic, samples, DAC_ALIGN_12B_R);
				sprintf(c, "%.2f",(float)frequency);
				strcat(c, " Hz");
				ILI9341_printText(c, 10,110, FCOLOR, BGCOLOR, 3);
			/*What if the measurement is beyond the thresholds*/

		}
		ctr++;

		if(frequency > 20000)
		{
			swp = 0;
			ILI9341_printText("Analysis completed!", 10, 100, FCOLOR, BGCOLOR, 2);
			ILI9341_printText("\"Down\"-Button to store." , 10, 120, FCOLOR, BGCOLOR,2);
		}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*"left" button*/
	if(GPIO_Pin == GPIO_PIN_0 && debounce())
	{
		call_ui();
		if(ui.frequency)
		{
			fcDigit-=1;
			if(fcDigit<0)
			{
				fcDigit = 4;
			}
		}
		if(ui.amplify)
		{
			acDigit-=1;
			if(acDigit<0)
			{
				acDigit = 3;
			}
		}
		call_ui();
	}

	/*"middle" button*/
	if(GPIO_Pin == GPIO_PIN_1 && debounce())
	{
		call_ui();
		pushedCtr = 0;
		longPress = 0;
		shortPress = 0;

		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 1)
		{
			pushedCtr += 1;
			if(pushedCtr > 1000000) //button was pressed for a long time
			{
				longPress = 1;
			}
			else if(pushedCtr > 500000 && pushedCtr < 1000000)
			{
				if(ui.amplify)
				{
					HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
					ILI9341_Fill(BGCOLOR);
					amplitude = 0;
					dc = 0;
					for(int i=3; i>=0;i-=1) //now let's bring all our fDigits together to form one frequency
					{
						amplitude=(amplitude+ (((uint16_t)aDigit[i])*pow(10,(3-i))));
					}
					clear_ui(&ui);
					ctrl = 1; //Control mode

					/*Create a sine with f=100 to control the amplitude*/
					frequency = 100;
					samples = SPS / frequency;
					TIM8->ARR = (HAL_RCC_GetHCLKFreq()/((TIM8->PSC+1)*samples*frequency));
					free(SineDynamic);
					SineDynamic = (uint16_t*) malloc(samples * sizeof(uint16_t));
					create_sine(&SineDynamic[0], samples, frequency);
					HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t * ) SineDynamic, samples, DAC_ALIGN_12B_R);
					potOneValue[1] = 127;
					amp = (float)amplitude/100;
					HAL_I2C_Master_Transmit(&hi2c1, 0x50, potOneValue, 2,10 );
					HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC_Measurement, ADC_SAMPLES);
					ILI9341_printText("adjusting gain", 30, 100, FCOLOR, BGCOLOR, 2);
				}
			}
			else if(pushedCtr > 100 && pushedCtr < 20000)
			{
				shortPress = 1;
			}

			if((ui.measure && shortPress))
			{
				HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC_Measurement, ADC_SAMPLES);
			}

			if((ui.frequency) && shortPress)
			{
				ILI9341_Fill(BGCOLOR);
				HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
				frequency = 0;
				for(int i=4; i>=0;i-=1) //now let's bring all our fDigits together.
				{
					frequency=frequency+(fDigit[i]*pow(10,(4-i)));
					dc = 0;
				}
				if(frequency == 0)
				{
					frequency = 20000;
					dc = 1;
				}
				samples = SPS / frequency;
				TIM8->ARR = (HAL_RCC_GetHCLKFreq()/((TIM8->PSC+1)*samples*frequency));
				free(SineDynamic);
				SineDynamic = (uint16_t*) malloc(samples * sizeof(uint16_t));
				create_sine(&SineDynamic[0], samples, frequency);
				HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t * ) SineDynamic, samples, DAC_ALIGN_12B_R);
				clear_ui(&ui);
				ui.measure = 1;
				if(swp)
				{
					clear_ui(&ui);
					ui.sweep = 1;
				}
				HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADC_Measurement, ADC_SAMPLES);
				call_ui();
			}
			if(ui.sweep && shortPress)
			{
				/*start sinusoidal at 100 Hz (later 10), start ADC. The logic will be in the ADC interrupt*/

			}
			if(longPress)
			{
				ILI9341_Fill(BGCOLOR);
				clear_ui(&ui);
				ui.start = 1;
			}

		}
		call_ui();
	}

	/*"up" button*/
	if(GPIO_Pin == GPIO_PIN_2 && debounce())
	{
		call_ui();
		if(ui.start == 1)
		{
			ILI9341_Fill(BGCOLOR);
			clear_ui(&ui);
			ui.amplify = 1;
		}
		if(ui.frequency)
		{
			fDigit[fcDigit]+=1;
			if(fcDigit == 0 && fDigit[fcDigit] > 2)
			{
				fDigit[fcDigit] = 0;
			}
			if(fDigit[0] ==2)
			{
				fDigit[0] = 2;
				fDigit[1] = 0;
				fDigit[2] = 0;
				fDigit[3] = 0;
				fDigit[4] = 0;
			}
			else if(fDigit[fcDigit]>9)
			{
				fDigit[fcDigit] = 0;
			}
		}
		if(ui.amplify)
		{
			aDigit[acDigit]+=1;
			if(acDigit == 0 && aDigit[acDigit] > 2)
			{
				aDigit[acDigit] = 0;
			}
			if(aDigit[0] ==2)
			{
				aDigit[0] = 2;
				aDigit[1] = 0;
				aDigit[2] = 0;
			}
			else if(aDigit[acDigit]>9)
			{
				aDigit[acDigit] = 0;
			}
		}
		call_ui();
	}

	/*"down" button*/
	if(GPIO_Pin == GPIO_PIN_3 && debounce())
	{
		call_ui();
		if(ui.start == 1)
		{
			ILI9341_Fill(BGCOLOR);
			clear_ui(&ui);
			ui.amplify = 1;
			swp = 1;
		}
		if(ui.frequency)
		{
			fDigit[fcDigit]-=1;
			if(fDigit[0]<0)
			{
				fDigit[fcDigit] = 2;
			}
			if(fDigit[0] >= 2)
			{
				fDigit[0] = 2;
				fDigit[1] = 0;
				fDigit[2] = 0;
				fDigit[3] = 0;
				fDigit[4] = 0;
			}

			if(fDigit[fcDigit]<0)
			{
				fDigit[fcDigit] = 9;
			}

		}
		if(ui.amplify)
		{
			aDigit[acDigit]-=1;
			if(aDigit[0]<0)
			{
				aDigit[acDigit] = 2;
			}
			if(aDigit[0] >= 2)
			{
				aDigit[0] = 2;
				aDigit[1] = 0;
				aDigit[2] = 0;
				aDigit[3] = 0;
			}
			if(aDigit[acDigit]<0)
			{
				aDigit[acDigit] = 9;
			}
		}
		if(ui.sweep)
		{
			//now it's time to write all that stuff to disc!
			if(!swp)
			{
				if(f_mount(&myFS,SDPath, 1) == FR_OK)
				{
					char myPath[] = "FREQRESP.TXT\0";
					if(f_open(&myFILE, myPath, FA_WRITE | FA_OPEN_APPEND) == FR_OK)
					{
						for(int ctr = 0; ctr < 2000 ;ctr++)
						{
							if(writeOut[ctr][2] !=0)
							{
								sprintf(c, "\n%.2f, %.2f, %.1f, %.2f",writeOut[ctr][0],writeOut[ctr][1], writeOut[ctr][2], writeOut[ctr][3]);
								f_puts(c, &myFILE);
							}
						}
					}

					f_close(&myFILE);

				}
			}

		}
		call_ui();
	}
	/*"right" button*/
	if(GPIO_Pin == GPIO_PIN_5 && debounce())
	{
		call_ui();
		if(ui.frequency)
		{
			fcDigit+=1;
			if(fcDigit > 4)
			{
				fcDigit = 0;
			}
		}
		if(ui.amplify)
		{
			acDigit+=1;
			if(acDigit > 3)
			{
				acDigit = 0;
			}
		}
		call_ui();
	}
}

/*the measurements are read*/


/*timer for screen printing*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)
	{
		debounceTime++;
	}
}

/*creates a sinusoid*/
void create_sine(uint16_t *sine, uint32_t samples, int frequency)
{
	if(dc)
	{
		for(int i = 0; i < samples; i++)
		{
			*(sine+i) = 4000;
		}
	}
	else if(frequency > 0.0) //it's a sinusoidal
	{
		for(int i = 0; i< samples; i++)
		{
			*(sine+i) = (uint16_t) (((sin(((i*2*PI)/samples) )+1)*(4096/2)*0.96)+80);
		}
	}

}

/*call ui screen*/
void call_ui(void)
{
	if(ui.start)
	{
		start_ui();
	}
	if(ui.frequency)
	{
		frequency_ui();
	}
	if(ui.sweep)
	{
		sweep_ui();
	}
	if(ui.amplify)
	{
		amplify_ui();
	}
	if(ui.measure)
	{
		measure_ui();
	}
}

/*reset the ui, i.e. the elements of the structure 'frame'*/
void clear_ui(UI *ui)
{
	ui->amplify = 0;
	ui->frequency = 0;
	ui->start = 0;
	ui->sweep = 0;
	ui->measure = 0;
}

/*print startscreen*/
void start_ui(void)
{
	ILI9341_Fill(BGCOLOR);
	ILI9341_printText("Choose:", 10, 20, FCOLOR, BGCOLOR, 2);
	ILI9341_printText("Single Frequency (Up)", 20, 60, FCOLOR, BGCOLOR, 2);
	ILI9341_printText("Frequency Sweep (Down)", 20, 90, FCOLOR, BGCOLOR, 2);
}

/*print screen for frequency choice*/
void frequency_ui(void)
{
	char c[5] = {0};
	ILI9341_Fill(BGCOLOR);
	sprintf(c, "%d%d%d%d%d",fDigit[0],fDigit[1],fDigit[2],fDigit[3],fDigit[4] );
	ILI9341_printText("Choose Frequency:", 10, 20, FCOLOR, BGCOLOR, 2);
	ILI9341_drawChar(50,110,c[0],FCOLOR,BGCOLOR,3);
	ILI9341_drawChar(70,110,c[1],FCOLOR,BGCOLOR,3);
	ILI9341_drawChar(90,110,c[2],FCOLOR,BGCOLOR,3);
	ILI9341_drawChar(110,110,c[3],FCOLOR,BGCOLOR,3);
	ILI9341_drawChar(130,110,c[4],FCOLOR,BGCOLOR,3);
	ILI9341_printText("Hz", 160, 110, FCOLOR, BGCOLOR, 3);
	ILI9341_printText("                    ", 50, 130, FCOLOR, BGCOLOR, 3);
	switch(fcDigit) /*underline the current digit*/
	{
	case 0: ILI9341_drawChar(50,130,'-',FCOLOR,BGCOLOR,3);
	break;
	case 1:	ILI9341_drawChar(70,130,'-',FCOLOR,BGCOLOR,3);
	break;
	case 2:	ILI9341_drawChar(90,130,'-',FCOLOR,BGCOLOR,3);
	break;
	case 3:	ILI9341_drawChar(110,130,'-',FCOLOR,BGCOLOR,3);
	break;
	case 4:	ILI9341_drawChar(130,130,'-',FCOLOR,BGCOLOR,3);
	break;
	}
}

/*print screen for adjusting gain*/
void amplify_ui(void)
{
	char c[4] = {0};
	ILI9341_Fill(BGCOLOR);
	sprintf(c, "%d%d%d%d",aDigit[0],aDigit[1],aDigit[2], aDigit[3]);
	ILI9341_printText("Choose Amplitude (Vpp):", 10, 20, FCOLOR, BGCOLOR, 2);
	ILI9341_drawChar(50,110,c[0],FCOLOR,BGCOLOR,3);
	ILI9341_drawChar(70,110,c[1],FCOLOR,BGCOLOR,3);
	ILI9341_drawChar(90,110,'.',FCOLOR,BGCOLOR,3);
	ILI9341_drawChar(110,110,c[2],FCOLOR,BGCOLOR,3);
	ILI9341_drawChar(130,110,c[3],FCOLOR,BGCOLOR,3);
	ILI9341_printText("Vpp", 160, 110, FCOLOR, BGCOLOR, 3);
	ILI9341_printText("                    ", 50, 135, FCOLOR, BGCOLOR, 3);
	switch(acDigit) /*underline the current digit*/
	{
	case 0: ILI9341_drawChar(50,135,'-',FCOLOR,BGCOLOR,3);
	break;
	case 1:	ILI9341_drawChar(70,135,'-',FCOLOR,BGCOLOR,3);
	break;
	case 2:	ILI9341_drawChar(110,135,'-',FCOLOR,BGCOLOR,3);
	break;
	case 3:	ILI9341_drawChar(130,135,'-',FCOLOR,BGCOLOR,3);
	break;
	}

}

/*ToDo: new screen for sweep results, showing the actual graph*/


/*print screen for sweep information*/
void sweep_ui(void)
{
	ILI9341_Fill(BGCOLOR);
	ILI9341_printText("Analysis started,", 10, 20, FCOLOR, BGCOLOR, 3);
	ILI9341_printText("please be patient.", 10, 60, FCOLOR, BGCOLOR, 2);
	if(!swp)
	{
		ILI9341_printText("Results saved." , 20, 140, FCOLOR, BGCOLOR,2);
	}
}

/*prints the measurement on the screen*/
void measure_ui(void)
{
	char c[10];
	char vpp[6] = "  VPP";
	char rms[6] = "  RMS";
	char hz[5] = "  Hz";
	char dC[4] = "  DC";
	ILI9341_Fill(BGCOLOR);
	ILI9341_printText("Results", 100, 20, FCOLOR, BGCOLOR, 3);
	if(dc)
	{
		ILI9341_printText(dC, 100, 60, FCOLOR, BGCOLOR, 2);
		ILI9341_printText("In: ", 30, 100, FCOLOR, BGCOLOR, 2);
		sprintf(c, "%.2f ",dc1);
		strcat(c, dC);
		ILI9341_printText(c, 100,100, FCOLOR, BGCOLOR, 2);

		ILI9341_printText("Out: ", 30, 160, FCOLOR, BGCOLOR, 2);
		sprintf(c, "%.2f ", dc0);
		strcat(c, dC);
		ILI9341_printText(c, 100,160, FCOLOR, BGCOLOR, 2);
	}
	else
	{
		sprintf(c, "%d", frequency);
		strcat(c, hz);
		ILI9341_printText(c, 100, 60, FCOLOR, BGCOLOR, 2);

		ILI9341_printText("In: ", 30, 100, FCOLOR, BGCOLOR, 2);
		sprintf(c, "%.2f ",rmsMean1);
		strcat(c, rms);
		ILI9341_printText(c, 100,100, FCOLOR, BGCOLOR, 2);
		sprintf(c, "%.2f",amp1);
		strcat(c, vpp);
		ILI9341_printText(c, 100,120, FCOLOR, BGCOLOR, 2);


		ILI9341_printText("Out: ", 30, 160, FCOLOR, BGCOLOR, 2);
		sprintf(c, "%.2f ", rmsMean0);
		strcat(c, rms);
		ILI9341_printText(c, 100,160, FCOLOR, BGCOLOR, 2);
		sprintf(c, "%.2f ", amp0);
		strcat(c, vpp);
		ILI9341_printText(c, 100,180, FCOLOR, BGCOLOR, 2);

	}
}

int debounce(void)
{
	static uint16_t dTime1, dTime2 = 0;
	dTime2 = dTime1;
	dTime1 = debounceTime;
	if(dTime1 - dTime2 >= DEBOUNCE_TIME)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif
/* USE_FULL_ASSERT */
/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
