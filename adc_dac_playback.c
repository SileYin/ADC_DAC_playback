/*********************************************************************************

Copyright(c) 2014-2016 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/
/*!
 * @file      adc_dac_playback.c
 * @brief     This example demonstrates the submission of DMA driven audio buffers
 *            to both the ADC and DAC.
 * @version:  $Revision: 31665 $
 * @date:     $Date: 2017-12-12 02:26:29 -0500 (Tue, 12 Dec 2017) $
 *
 * @details
 *            This is the primary source file for ADC / DAC play-back example that shows
 *            how to submit DMA driven audio buffers to both the ADC and DAC .
 *
 */

/*=============  I N C L U D E S   =============*/

#include <sys/platform.h>
/* SPU Manager includes */
#include <services/spu/adi_spu.h>
#include <services/gpio/adi_gpio.h>
#include <drivers/adc/adau1979/adi_adau1979.h>
#include <drivers/dac/adau1962a/adi_adau1962a.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "adc_dac_playback.h"
#include "adi_initialize.h"
#include <filter.h>
#include <complex.h>
#include <SRU.h>
#include <cycles.h>
#include <stats.h>
#include <stdlib.h>

/*==============  D E F I N E S  ===============*/

/*=============  D A T A  =============*/

/* Twi  */
uint32_t TwiMemory[ADI_TWI_MEMORY_SIZE];
/* Gpio */
uint32_t GpioMemory[ADI_GPIO_CALLBACK_MEM_SIZE];
uint32_t gpioMaxCallbacks;

/* ADAU1979 ADC */
static ADI_ADAU1979_HANDLE phAdau1979;
uint32_t Adau1979Memory[ADI_ADAU1979_MEMORY_SIZE];
/* ADAU1979 Sport */
uint32_t Adau1979SportMemory[ADI_SPORT_DMA_MEMORY_SIZE];

/* ADAU1962A DAC DATA */
static ADI_ADAU1962A_HANDLE phAdau1962a;
uint32_t Adau1962aMemory[ADI_ADAU1962A_MEMORY_SIZE];
/* ADAU1962A Sport */
uint32_t Adau1962aSportMemory[ADI_SPORT_DMA_MEMORY_SIZE];

/* Counter to keep track of number of ADC buffers processed */
volatile uint32_t AdcCount = 0u;
/* Counter to keep track of number of DAC buffers processed */
volatile uint32_t DacCount = 0u;

volatile uint32_t AdcCount100 = 0u;

/* DAC buffer pointer */
volatile void *pGetADC = NULL;
/* ADC buffer pointer */
volatile void *pGetDAC = NULL;

void *pADC;

/* Flag to register callback error */
volatile bool bEventError = false;

/* Adc linear buffer that is divided into 2 sub buffers; ping and pong  */
#pragma align 4
int8_t AdcBuf[AUDIO_BUFFER_SIZE * 2];

/* Dac linear buffer that is divided into 2 sub buffers; ping and pong  */
#pragma align 4
int8_t DacBuf[AUDIO_BUFFER_SIZE * 2];


int32_t CorrTable[CORRELATION_SIZE];

#if defined(__ADSPBF707_FAMILY__) || defined(__ADSP215xx__)
/* Memory required for the SPU operation */
static uint8_t  SpuMemory[ADI_SPU_MEMORY_SIZE];

/* SPU handle */
static ADI_SPU_HANDLE hSpu;
#endif

/*=============  L O C A L    F U N C T I O N    P R O T O T Y P E S =============*/
/* Initialize GPIO and reset peripherals */
uint32_t    GpioInit(void);
/* Initializes ADC */
uint32_t    Adau1979Init(void);
/* Initializes DAC */
uint32_t    Adau1962aInit(void);
/* Submit buffers to ADC */
uint32_t    Adau1979SubmitBuffers(void);
/* Submit buffers to DAC */
uint32_t    Adau1962aSubmitBuffers(void);
/* Process audio buffers */
uint32_t 	ProcessBuffers(void);
/* Generate sine wave test tone */
uint32_t 	Generate_SineWave(uint32_t Frequency, uint32_t SampleRate, void * pBuffer, uint32_t samples, uint32_t numchannels);
/* Perform a correlation on a specified buffer */
uint32_t 	Correlation(int32_t * pBuffer, uint32_t samples, uint32_t stride, int32_t * pCoorelationTable);
/* Check a specified correlation table for a reference frequency */
uint32_t	Check_Correlation(uint32_t ref_freq, uint32_t samples, int32_t * pCoorelationTable);
/*=============  C A L L B A C K    F U N C T I O N    P R O T O T Y P E S =============*/

/* ADC callback */
void AdcCallback(void *pCBParam, uint32_t nEvent, void *pArg);
/* DAC callback */
void DacCallback(void *pCBParam, uint32_t nEvent, void *pArg);
float Fir_Dir_I(int firLength,int firBufferCnt,float firFilterCoef[],float firFilterBuffer[]);
float Upsampling_FIR(int firLength,int firBufferCnt,int firCoeffCnt,float firFilterCoef[],float firFilterBuffer[]);
void fft_filter(float input[],float output[]);
void block_fft_filter(float input[],float output[]);
float vector_norm(float vector[],int length);
float cross_norm(float x[],float y[],int length);
int find_max(const float x[], int length);
void NLMSSGD(float x[],float fit[],float error,int length,float stepSize);
void ps_update(float ps[], complex_float xf[]);
void noise_reduction(float np[], float ps[], complex_float xf[], complex_float yf[]);
void array_copy(float x[],float y[]);
void spectral_smooth(float gain[],float win[],int fftLength,int winLength,int halfWin);
void filter_upd(int filterLength,int filterCnt,float filter[],float zBuffer[],float error);
/*=============  E X T E R N A L    F U N C T I O N    P R O T O T Y P E S =============*/

/* Configures soft switches */
extern void ConfigSoftSwitches(void);

/*=============  C O D E  =============*/

/*
 * Main function
 */
int main()
{
	/**
	 * Initialize managed drivers and/or services that have been added to
	 * the project.
	 * @return zero on success
	 */
	uint32_t Result = 0u;
	uint32_t i;

	/* configure System Event Controller SEC and Signal Routing Unit SRU */
	adi_initComponents();

	/* Software Switch Configuration for the EZ-Board */
	ConfigSoftSwitches();

	/* Initialize GPIO for ADC/DAC reset control */
	if(Result == 0u)
	{
		Result = GpioInit();
	}

/* ADSP-BF707 or ADSP-SC589 Processor family */
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)

    /* Initialize SPU Service */
    if(adi_spu_Init(0u, SpuMemory, NULL, NULL, &hSpu) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to initialize SPU service\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4A to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4A_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4A\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4B to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4B_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4B\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4A DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4A_DMA10_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4A DMA\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 4B DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_4B_DMA11_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 4B DMA\n");
        return (ADI_SPU_FAILURE);
    }

#elif defined(__ADSPSC573_FAMILY__)

    /* Initialize SPU Service */
    if(adi_spu_Init(0u, SpuMemory, NULL, NULL, &hSpu) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to initialize SPU service\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 2A to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_2A_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 2A\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 2B to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_2B_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 2B\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 2A DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_2A_DMA4_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 2A DMA\n");
        return (ADI_SPU_FAILURE);
    }

    /* Make SPORT 2B DMA to generate secure transactions */
    if(adi_spu_EnableMasterSecure(hSpu, SPORT_2B_DMA5_SPU_PID, true) != ADI_SPU_SUCCESS)
    {
        DBG_MSG("Failed to enable Master secure for SPORT 2B DMA\n");
        return (ADI_SPU_FAILURE);
    }

#else
#error "processor not supported"
#endif

/* Generate sine wave test tone for ping and pong buffers */
#ifndef PLAYBACK
	/* generate sine wave */
	if(Result == 0u)
	{
		for( i = 0u; i < NUM_CHANNELS ; i++ )
		{
			Result = Generate_SineWave(REFERENCE_FREQ, SAMPLE_RATE, &DacBuf[(AUDIO_BUFFER_SIZE * 0u) + (4u*i)], SAMPLES_PER_PERIOD * 4u, NUM_CHANNELS);
			if(Result != 0u)
			{
				/* there has been an error */
				break;
			}
			Result = Generate_SineWave(REFERENCE_FREQ, SAMPLE_RATE, &DacBuf[(AUDIO_BUFFER_SIZE * 1u) + (4u*i)], SAMPLES_PER_PERIOD * 4u, NUM_CHANNELS);
			if(Result != 0u)
			{
				/* there has been an error */
				break;
			}
		}
	}
#endif

	/* Initialize ADAU1979 */
	if(Result == 0u)
	{
		Result = Adau1979Init();
	}

	/* Initialize ADAU1962A */
	if(Result == 0u)
	{
		Result = Adau1962aInit();
	}

	/* Submit ADC buffers */
	if(Result == 0u)
	{
		Result = Adau1979SubmitBuffers();
	}

	/* Submit DAC buffers */
	if(Result == 0u)
	{
		Result = Adau1962aSubmitBuffers();
	}

	/* Enable data flow for only the ADC and DAC */

	if((uint32_t)adi_adau1962a_Enable(phAdau1962a, true) != 0u)
	{
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_adau1979_Enable(phAdau1979, true) != 0u)
	{
		/* return error */
		return 1u;
	}

	if(Result == 0u)
	{
	    /* Audio loopback */
		while(1)
		{
		    /* Process audio buffers */
			Result = ProcessBuffers();

			/* IF (Error) */
			if(Result != 0u)
			{
			    /* exit loopback */
				break;
			}

			/* check if an error has been detected in callback */
            if(bEventError)
			{
			    /* there has been an error returned in the callback */
				Result =1u;
				break;
            }

			/* Callback count limit reached */
			if (0/*(AdcCount >CALLBACK_COUNT) || (DacCount >CALLBACK_COUNT)*/)
			{

				/* Disable ADC data flow */
				if((uint32_t)adi_adau1979_Enable(phAdau1979, false) != 0u)
				{
					/* return error */
					Result = 1u;
				}

				/* Disable DAC data flow */
				adi_adau1962a_Enable(phAdau1962a, false);

				/* Close ADC device */
				adi_adau1979_Close(phAdau1979);

				/* Close DAC device */
				adi_adau1962a_Close(phAdau1962a);
				/* while loop break */
				break;
			}
		}
    }
#ifndef PLAYBACK
	if(Result == 0u)
	{
		for( i = 0u; i < NUM_CHANNELS ; i++ )
		{
			/* perform correlation of input buffer */
			Result = Correlation((int32_t *)&AdcBuf[(AUDIO_BUFFER_SIZE * 0u) + (NUM_CHANNELS*i)], CORRELATION_SIZE, 4u, &CorrTable[0]);
			if(Result != 0u)
			{
				/* there has been an error */
				break;
			}
			Result = Check_Correlation(REFERENCE_FREQ, CORRELATION_SIZE, &CorrTable[0]);
			if(Result != 0u)
			{
				/* there has been an error */
				break;
			}
		}
	}
#endif

	if (Result == 0u)
	{
		printf("All done\n");
	}
	else
	{
		printf("Error\n");
	}

	return 0;
}

/*
 * Initializes GPIO service
 * A GPIO line is used to control reset of the ADC and DAC devices
 */
uint32_t GpioInit(void)
{
	uint32_t Result = 0u;
	/* Loop variable */
	volatile uint32_t i;

	if((uint32_t)adi_gpio_Init((void*)GpioMemory, ADI_GPIO_CALLBACK_MEM_SIZE, &gpioMaxCallbacks) != 0u)
	{
		/* return error */
		return 1u;
	}

#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)
	if((uint32_t)adi_gpio_SetDirection(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14, ADI_GPIO_DIRECTION_OUTPUT) != 0u)
	{
		/* return error */
		return 1u;
	}
	/* bring reset low */
	if((uint32_t)adi_gpio_Clear(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* delay */
	for (i = DELAY_COUNT; i ; i --);

	/* bring reset high */
	if((uint32_t)adi_gpio_Set(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14) != 0u)
	{
		/* return error */
		return 1u;
	}
	
#elif defined(__ADSPSC573_FAMILY__)
	if((uint32_t)adi_gpio_SetDirection(ADI_GPIO_PORT_A, ADI_GPIO_PIN_6, ADI_GPIO_DIRECTION_OUTPUT) != 0u)
	{
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_gpio_SetDirection(ADI_GPIO_PORT_A, ADI_GPIO_PIN_7, ADI_GPIO_DIRECTION_OUTPUT) != 0u)
	{
		/* return error */
		return 1u;
	}
	
	/* bring reset low */
	if((uint32_t)adi_gpio_Clear(ADI_GPIO_PORT_A, ADI_GPIO_PIN_6) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* bring reset low */
	if((uint32_t)adi_gpio_Clear(ADI_GPIO_PORT_A, ADI_GPIO_PIN_7) != 0u)
	{
		/* return error */
		return 1u;
	}
	
	/* delay */
	for (i = DELAY_COUNT; i ; i --);

	/* bring reset high */
	if((uint32_t)adi_gpio_Set(ADI_GPIO_PORT_A, ADI_GPIO_PIN_6) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* bring reset high */
	if((uint32_t)adi_gpio_Set(ADI_GPIO_PORT_A, ADI_GPIO_PIN_7) != 0u)
	{
		/* return error */
		return 1u;
	}
	
#else
#error "processor not supported"
#endif

	/* delay */
	for (i = DELAY_COUNT; i ; i --);

	return Result;
}

/*
 * Opens and initializes ADAU1979 ADC Device.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1979Init(void)
{
	uint32_t Result = 0u;
	/* Instance to submit SPORT configuration */
	ADI_ADAU1979_SPORT_CONFIG   SportConfig;
	/* Instance to submit TWI configuration */
    ADI_ADAU1979_TWI_CONFIG     TwiConfig;

	/* open ADAU1979 instance */
	if((uint32_t)adi_adau1979_Open(0u,
#ifdef TDM_MODE
            					   ADI_ADAU1979_SERIAL_MODE_TDM4,
#else
            					   ADI_ADAU1979_SERIAL_MODE_STEREO,
#endif
            					   &Adau1979Memory,
            					   ADI_ADAU1979_MEMORY_SIZE,
            					   &phAdau1979) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_Open failed\n");
		/* return error */
		return 1u;
	}

	/* TWI parameters required to open/configure TWI */
	TwiConfig.TwiDevNum 	= 0u;
	TwiConfig.TwiDevMemSize	= ADI_TWI_MEMORY_SIZE;
	TwiConfig.pTwiDevMem 	= &TwiMemory;
	TwiConfig.eTwiAddr 		= ADI_ADAU1979_TWI_ADDR_11;

	if((uint32_t)adi_adau1979_ConfigTwi(phAdau1979, &TwiConfig) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_ConfigTwi failed\n");
		/* return error */
		return 1u;
	}

	/* SPORT parameters required to open/configure TWI */
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)	
	SportConfig.SportDevNum 	= 4u;
#elif defined(__ADSPSC573_FAMILY__)
    SportConfig.SportDevNum 	= 2u;
#else	
#error "processor not defined"
#endif
	SportConfig.SportDevMemSize	= ADI_SPORT_DMA_MEMORY_SIZE;
	SportConfig.pSportDevMem 	= &Adau1979SportMemory;
	SportConfig.eSportChnl	    = ADI_ADAU1979_SPORT_A;
	SportConfig.eSportPri	    = ADI_ADAU1979_SERIAL_PORT_DSDATA1;
#ifdef TDM_MODE
	SportConfig.eSportSec	    = ADI_ADAU1979_SERIAL_PORT_NONE;
#else
	SportConfig.eSportSec	    = ADI_ADAU1979_SERIAL_PORT_DSDATA2;
#endif
	SportConfig.bLsbFirst		= true;

	if((uint32_t)adi_adau1979_ConfigSport(phAdau1979, &SportConfig) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_ConfigSport failed\n");
		/* return error */
		return 1u;
	}

	/* ADC is a master source of SPORT clk and FS, MCLK 25.576 MHz and PLL used MCLK */
	if((uint32_t)adi_adau1979_ConfigPllClk(phAdau1979,
										   LR_B_CLK_MASTER_1979,
			                               ADI_ADAU1979_MCLK_IN_FREQ_24576000HZ,
			                               ADI_ADAU1979_PLL_SOURCE_MCLK) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_ConfigPllClk failed\n");
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_adau1979_ConfigSerialClk(phAdau1979,
            								  BCLK_RISING_1979,
            								  LRCLK_HI_LO_1979) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_ConfigSerialClk failed\n");
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_adau1979_SetSampleRate(phAdau1979, ADI_ADAU1979_SAMPLE_RATE_48000HZ) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_SetSampleRate failed\n");
		/* return error */
		return 1u;
	}
	if((uint32_t)adi_adau1979_SetWordWidth(phAdau1979, ADI_ADAU1979_WORD_WIDTH_24) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_SetWordWidth failed\n");
		/* return error */
		return 1u;
	}

	if((uint32_t)adi_adau1979_RegisterCallback(phAdau1979, AdcCallback, NULL) != 0u)
	{
		printf ("ADAU1979: adi_adau1979_RegisterCallback failed\n");
		/* return error */
		return 1u;
	}

	return Result;
}

/*
 * Opens and initializes ADAU1962A DAC Device.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1962aInit(void)
{
    ADI_ADAU1962A_RESULT        eResult;
    ADI_ADAU1962A_TWI_CONFIG    TwiConfig;
    ADI_ADAU1962A_SPORT_CONFIG  SportConfig;

	/* Open ADAU1962A device instance */
	if((eResult = adi_adau1962a_Open(0u,
#ifdef TDM_MODE
            						 ADI_ADAU1962A_SERIAL_MODE_TDM4,
#else
             					     ADI_ADAU1962A_SERIAL_MODE_STEREO,
#endif
            					     &Adau1962aMemory,
            					     ADI_ADAU1962A_MEMORY_SIZE,
            					     &phAdau1962a)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to open ADAU1962A device instance, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* TWI parameters required to open/configure TWI */
	TwiConfig.TwiDevNum 	= 0u;
	TwiConfig.eTwiAddr 		= ADI_ADAU1962A_TWI_ADDR_04;
	TwiConfig.TwiDevMemSize	= ADI_TWI_MEMORY_SIZE;
	TwiConfig.pTwiDevMem 	= &TwiMemory;

    /* Configure TWI */
	if ((eResult = adi_adau1962a_ConfigTwi (phAdau1962a, &TwiConfig)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure TWI, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* SPORT parameters required to open/configure SPORT */
#if defined(__ADSPBF707_FAMILY__) || defined(__ADSPSC589_FAMILY__)	
	SportConfig.SportDevNum 	= 4u;
#elif defined(__ADSPSC573_FAMILY__)
    SportConfig.SportDevNum 	= 2u;
#else	
#error "processor not defined"
#endif
	SportConfig.eSportChnl	    = ADI_ADAU1962A_SPORT_B;
	SportConfig.eSportPri	    = ADI_ADAU1962A_SERIAL_PORT_DSDATA1;
#ifdef TDM_MODE
	SportConfig.eSportSec	    = ADI_ADAU1962A_SERIAL_PORT_NONE;
#else
	SportConfig.eSportSec	    = ADI_ADAU1962A_SERIAL_PORT_DSDATA2;
#endif
	SportConfig.SportDevMemSize	= ADI_SPORT_DMA_MEMORY_SIZE;
	SportConfig.pSportDevMem 	= &Adau1962aSportMemory;

    /* Configure SPORT */
	if ((eResult = adi_adau1962a_ConfigSport (phAdau1962a, &SportConfig)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure SPORT, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* DAC Master Power-up */
	if ((eResult = adi_adau1962a_ConfigDacPwr (phAdau1962a,
											   ADI_ADAU1962A_CHNL_DAC_MSTR,
											   ADI_ADAU1962A_DAC_PWR_LOW,
											   true)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure DAC power, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

    /*
     * Configure PLL clock - DAC is clock master and drives SPORT clk and FS
     * MCLK 24.576 MHz and PLL uses MCLK
     */
	if ((eResult = adi_adau1962a_ConfigPllClk (phAdau1962a,
	                                           ADAU1962A_MCLK_IN,
	                                           ADI_ADAU1962A_MCLK_SEL_PLL,
	                                           ADI_ADAU1962A_PLL_IN_MCLKI_XTALI)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure PLL clock, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

    /*
     * Configure serial data clock
     * DAC as clock master, External BCLK, Latch on raising edge
     * LRCLK at 50% duty cycle, MSB first, Left channel at LRCLK low
     */
	if ((eResult = adi_adau1962a_ConfigSerialClk (phAdau1962a,
	                                              LR_B_CLK_MASTER_1962,
	                                              false,
	                                              BCLK_RISING_1962,
/* pulse mode - true */
#ifdef TDM_MODE
	                                              true,
#else
	                                              false,
#endif
	                                              false,
	                                              LRCLK_HI_LO_1962)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure serial data clock, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* Power-up PLL */
	if ((eResult = adi_adau1962a_ConfigBlockPwr (phAdau1962a,
	                                             false,
	                                             true,
	                                             true)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to Power-up PLL, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* Configure Sample rate */
	if ((eResult = adi_adau1962a_SetSampleRate (phAdau1962a, SAMPLE_RATE)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure Sample rate, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* Configure Word width */
	if ((eResult = adi_adau1962a_SetWordWidth (phAdau1962a,
	                                           ADI_ADAU1962A_WORD_WIDTH_24)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to configure word width, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	/* Register callback */
	if ((eResult = adi_adau1962a_RegisterCallback (phAdau1962a,
												   DacCallback,
	                                               NULL)) != ADI_ADAU1962A_SUCCESS)
	{
		printf ("ADAU1962A: Failed to register callback, Error Code: 0x%08X\n", eResult);
		/* return error */
		return 1u;
	}

	return 0u;
}

/*
 * Submits ping-pong buffers to ADC and enables ADC data flow.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1979SubmitBuffers(void)
{
	uint32_t Result = 0u;

	/* submit ping buffer */
	if((uint32_t)adi_adau1979_SubmitBuffer(phAdau1979, &AdcBuf[AUDIO_BUFFER_SIZE * 0u], AUDIO_BUFFER_SIZE) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* submit pong buffer */
	if((uint32_t)adi_adau1979_SubmitBuffer(phAdau1979, &AdcBuf[AUDIO_BUFFER_SIZE * 1u], AUDIO_BUFFER_SIZE) != 0u)
	{
		/* return error */
		return 1u;
	}

	return Result;
}

/*
 * Submits ping-pong buffers to DAC and enables ADC data flow.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */
uint32_t Adau1962aSubmitBuffers(void)
{
	uint32_t Result = 0u;

	/* submit ping buffer */
	if((uint32_t)adi_adau1962a_SubmitBuffer(phAdau1962a, &DacBuf[AUDIO_BUFFER_SIZE * 0u], AUDIO_BUFFER_SIZE) != 0u)
	{
		/* return error */
		return 1u;
	}

	/* submit pong buffer */
	if((uint32_t)adi_adau1962a_SubmitBuffer(phAdau1962a, &DacBuf[AUDIO_BUFFER_SIZE * 1u], AUDIO_BUFFER_SIZE) != 0u)
	{
		/* return error */
		return 1u;
	}

	return Result;
}

/*
 * Processes audio buffers that are processed by ADC and DAC driver.
 *
 * Parameters
 *  None
 *
 * Returns
 *  0 if success, other values for error
 *
 */

//int delay[CHANNEL_DELAY_LENGTH] = {0};
//int delaysample = 0;
//float firFilter[FIR_LENGTH] = {0.000184652111981946,0.000172060720270279,0.000155479089132623,0.000135017666726392,0.000110820559487155,8.30767117760908e-05,5.20329293476592e-05,1.80078572120486e-05,-1.85941098522255e-05,-5.72700929501979e-05,-9.74083158054774e-05,-0.000138281138857604,-0.000179042591980376,-0.000218731326307503,-0.000256279722067907,-0.000290529658146971,-0.000320255177655068,-0.000344191981386524,-0.000361073358622208,-0.000369671834486711,-0.000368845488108770,-0.000357587589655958,-0.000335077930347617,-0.000300733990637841,-0.000254259919664508,-0.000195691193982915,-0.000125432793745872,-4.42887856765214e-05,4.65186624681518e-05,0.000145342591523860,0.000250118338979431,0.000358385483006711,0.000467325276575332,0.000573813707529231,0.000674489781787521,0.000765838064149210,0.000844283942729003,0.000906299528174034,0.000948517577390292,0.000967850363312243,0.000961610016267600,0.000927626556257651,0.000864359634429309,0.000770999918892972,0.000647556104335244,0.000494923702389909,0.000314932082223497,0.000110366675748107,-0.000115036167601541,-0.000356623506738678,-0.000608890361892515,-0.000865574677729833,-0.00111978195935832,-0.00136413789222426,-0.00159096630153176,-0.00179248887687128,-0.00196104221292393,-0.00208930692674283,-0.00217054293429357,-0.00219882442960190,-0.00216926773191961,-0.00207824496885745,-0.00192357656096503,-0.00170469567511384,-0.00142277822396951,-0.00108083260456720,-0.000683744182156847,-0.000238270521591466,0.000247015472769983,0.000761836052254797,0.00129437289645442,0.00183148383330939,0.00235896917221520,0.00286188331939962,0.00332488589896818,0.00373262524975578,0.00407014594857177,0.00432331096342073,0.00447922820121388,0.00452667061420362,0.00445647869316644,0.00426193412209407,0.00393909361054875,0.00348707245994128,0.00290826825483435,0.00220851618787099,0.00139716890697601,0.000487095388294971,-0.000505404846982352,-0.00156077388047877,-0.00265645353248586,-0.00376722696285144,-0.00486563906049914,-0.00592248884011102,-0.00690738506344727,-0.00778935446004032,-0.00853749028906621,-0.00912162760828086,-0.00951303053862114,-0.00968507606918517,-0.00961391856370975,-0.00927911912448940,-0.00866422435166720,-0.00775727980391558,-0.00655126460971450,-0.00504443517590555,-0.00324056776169545,-0.00114909179274841,0.00121489186552048,0.00383071092762648,0.00667234059916592,0.00970873926258864,0.0129042961094983,0.0162193832507729,0.0196110023314246,0.0230335133719509,0.0264394315106708,0.0297802755850290,0.0330074511097090,0.0360731492227420,0.0389312426052598,0.0415381592534629,0.0438537152993104,0.0458418888349034,0.0474715178790767,0.0487169072072236,0.0495583307107428,0.0499824182154387,0.0499824182154387,0.0495583307107428,0.0487169072072236,0.0474715178790767,0.0458418888349034,0.0438537152993104,0.0415381592534629,0.0389312426052598,0.0360731492227420,0.0330074511097090,0.0297802755850290,0.0264394315106708,0.0230335133719509,0.0196110023314246,0.0162193832507729,0.0129042961094983,0.00970873926258864,0.00667234059916592,0.00383071092762648,0.00121489186552048,-0.00114909179274841,-0.00324056776169545,-0.00504443517590555,-0.00655126460971450,-0.00775727980391558,-0.00866422435166720,-0.00927911912448940,-0.00961391856370975,-0.00968507606918517,-0.00951303053862114,-0.00912162760828086,-0.00853749028906621,-0.00778935446004032,-0.00690738506344727,-0.00592248884011102,-0.00486563906049914,-0.00376722696285144,-0.00265645353248586,-0.00156077388047877,-0.000505404846982352,0.000487095388294971,0.00139716890697601,0.00220851618787099,0.00290826825483435,0.00348707245994128,0.00393909361054875,0.00426193412209407,0.00445647869316644,0.00452667061420362,0.00447922820121388,0.00432331096342073,0.00407014594857177,0.00373262524975578,0.00332488589896818,0.00286188331939962,0.00235896917221520,0.00183148383330939,0.00129437289645442,0.000761836052254797,0.000247015472769983,-0.000238270521591466,-0.000683744182156847,-0.00108083260456720,-0.00142277822396951,-0.00170469567511384,-0.00192357656096503,-0.00207824496885745,-0.00216926773191961,-0.00219882442960190,-0.00217054293429357,-0.00208930692674283,-0.00196104221292393,-0.00179248887687128,-0.00159096630153176,-0.00136413789222426,-0.00111978195935832,-0.000865574677729833,-0.000608890361892515,-0.000356623506738678,-0.000115036167601541,0.000110366675748107,0.000314932082223497,0.000494923702389909,0.000647556104335244,0.000770999918892972,0.000864359634429309,0.000927626556257651,0.000961610016267600,0.000967850363312243,0.000948517577390292,0.000906299528174034,0.000844283942729003,0.000765838064149210,0.000674489781787521,0.000573813707529231,0.000467325276575332,0.000358385483006711,0.000250118338979431,0.000145342591523860,4.65186624681518e-05,-4.42887856765214e-05,-0.000125432793745872,-0.000195691193982915,-0.000254259919664508,-0.000300733990637841,-0.000335077930347617,-0.000357587589655958,-0.000368845488108770,-0.000369671834486711,-0.000361073358622208,-0.000344191981386524,-0.000320255177655068,-0.000290529658146971,-0.000256279722067907,-0.000218731326307503,-0.000179042591980376,-0.000138281138857604,-9.74083158054774e-05,-5.72700929501979e-05,-1.85941098522255e-05,1.80078572120486e-05,5.20329293476592e-05,8.30767117760908e-05,0.000110820559487155,0.000135017666726392,0.000155479089132623,0.000172060720270279,0.000184652111981946};
//float firBuffer[FIR_LENGTH] = {0};
//int firBufferCounter = 0;
/* anti-aliasing filter */
//float aaFilter[FIR_LENGTH] = {4.593977e-07,-9.454628e-06,9.574093e-06,5.198642e-06,-1.278898e-06,-6.819917e-06,-6.744686e-06,3.668798e-07,9.004946e-06,1.031485e-05,8.806741e-07,-1.200659e-05,-1.540756e-05,-3.291519e-06,1.529777e-05,2.225877e-05,7.547501e-06,-1.839872e-05,-3.105040e-05,-1.438679e-05,2.063823e-05,4.181248e-05,2.460096e-05,-2.109809e-05,-5.433395e-05,-3.898162e-05,1.858750e-05,6.808335e-05,5.823345e-05,-1.165815e-05,-8.211931e-05,-8.286294e-05,-1.353038e-06,9.502007e-05,1.130399e-04,2.224351e-05,-1.048332e-04,-1.484474e-04,-5.281834e-05,1.090594e-04,1.881273e-04,9.472047e-05,-1.046841e-04,-2.303281e-04,-1.492193e-04,8.826378e-05,2.723817e-04,2.169676e-04,-5.607744e-05,-3.106164e-04,-2.977356e-04,4.352437e-06,3.403331e-04,3.901421e-04,7.044818e-05,-3.558580e-04,-4.913974e-04,-1.712673e-04,3.506856e-04,5.970939e-04,3.000463e-04,-3.177281e-04,-7.010518e-04,-4.572873e-04,2.496621e-04,7.952636e-04,6.416099e-04,-1.393857e-04,-8.699583e-04,-8.493356e-04,-1.942261e-05,9.137821e-04,1.074147e-03,2.316638e-04,-9.141794e-04,-1.306729e-03,-5.001960e-04,8.578248e-04,1.534759e-03,8.250815e-04,-7.313223e-04,-1.742811e-03,-1.202963e-03,5.219373e-04,1.912596e-03,1.626466e-03,-2.184892e-04,-2.023321e-03,-2.083711e-03,-1.877196e-04,2.052257e-03,2.557984e-03,7.019350e-04,-1.975470e-03,-3.027565e-03,-1.325032e-03,1.768699e-03,3.465768e-03,2.052648e-03,-1.408311e-03,-3.841134e-03,-2.874464e-03,8.722992e-04,4.117780e-03,3.773624e-03,-1.412031e-04,-4.255832e-03,-4.726337e-03,-8.011249e-04,4.211810e-03,5.701625e-03,1.967043e-03,-3.938792e-03,-6.661165e-03,-3.365103e-03,3.386064e-03,7.559073e-03,5.000710e-03,-2.497775e-03,-8.341373e-03,-6.877821e-03,1.209768e-03,8.944635e-03,9.002290e-03,5.569221e-04,-9.292825e-03,-1.138803e-02,-2.909001e-03,9.290341e-03,1.406851e-02,6.006976e-03,-8.806729e-03,-1.711963e-02,-1.012079e-02,7.641636e-03,2.070963e-02,1.576052e-02,-5.436877e-03,-2.522540e-02,-2.403147e-02,1.418896e-03,3.166255e-02,3.784098e-02,6.575577e-03,-4.328675e-02,-6.792964e-02,-2.808988e-02,7.924307e-02,2.112636e-01,3.017091e-01,3.017091e-01,2.112636e-01,7.924307e-02,-2.808988e-02,-6.792964e-02,-4.328675e-02,6.575577e-03,3.784098e-02,3.166255e-02,1.418896e-03,-2.403147e-02,-2.522540e-02,-5.436877e-03,1.576052e-02,2.070963e-02,7.641636e-03,-1.012079e-02,-1.711963e-02,-8.806729e-03,6.006976e-03,1.406851e-02,9.290341e-03,-2.909001e-03,-1.138803e-02,-9.292825e-03,5.569221e-04,9.002290e-03,8.944635e-03,1.209768e-03,-6.877821e-03,-8.341373e-03,-2.497775e-03,5.000710e-03,7.559073e-03,3.386064e-03,-3.365103e-03,-6.661165e-03,-3.938792e-03,1.967043e-03,5.701625e-03,4.211810e-03,-8.011249e-04,-4.726337e-03,-4.255832e-03,-1.412031e-04,3.773624e-03,4.117780e-03,8.722992e-04,-2.874464e-03,-3.841134e-03,-1.408311e-03,2.052648e-03,3.465768e-03,1.768699e-03,-1.325032e-03,-3.027565e-03,-1.975470e-03,7.019350e-04,2.557984e-03,2.052257e-03,-1.877196e-04,-2.083711e-03,-2.023321e-03,-2.184892e-04,1.626466e-03,1.912596e-03,5.219373e-04,-1.202963e-03,-1.742811e-03,-7.313223e-04,8.250815e-04,1.534759e-03,8.578248e-04,-5.001960e-04,-1.306729e-03,-9.141794e-04,2.316638e-04,1.074147e-03,9.137821e-04,-1.942261e-05,-8.493356e-04,-8.699583e-04,-1.393857e-04,6.416099e-04,7.952636e-04,2.496621e-04,-4.572873e-04,-7.010518e-04,-3.177281e-04,3.000463e-04,5.970939e-04,3.506856e-04,-1.712673e-04,-4.913974e-04,-3.558580e-04,7.044818e-05,3.901421e-04,3.403331e-04,4.352437e-06,-2.977356e-04,-3.106164e-04,-5.607744e-05,2.169676e-04,2.723817e-04,8.826378e-05,-1.492193e-04,-2.303281e-04,-1.046841e-04,9.472047e-05,1.881273e-04,1.090594e-04,-5.281834e-05,-1.484474e-04,-1.048332e-04,2.224351e-05,1.130399e-04,9.502007e-05,-1.353038e-06,-8.286294e-05,-8.211931e-05,-1.165815e-05,5.823345e-05,6.808335e-05,1.858750e-05,-3.898162e-05,-5.433395e-05,-2.109809e-05,2.460096e-05,4.181248e-05,2.063823e-05,-1.438679e-05,-3.105040e-05,-1.839872e-05,7.547501e-06,2.225877e-05,1.529777e-05,-3.291519e-06,-1.540756e-05,-1.200659e-05,8.806741e-07,1.031485e-05,9.004946e-06,3.668798e-07,-6.744686e-06,-6.819917e-06,-1.278898e-06,5.198642e-06,9.574093e-06,-9.454628e-06,4.593977e-07};
//float aaFilter[FIR_LENGTH] = {-8.45011304875402e-05,-0.00207925902605286,-0.000227768927611145,-0.000203831166464738,-4.23373398720898e-05,0.000139638083216722,0.000301322189111397,0.000399146152793842,0.000403778285772824,0.000306018107911122,0.000123551936096403,-0.000104469482676730,-0.000322747046191756,-0.000476023611671503,-0.000519910225057264,-0.000435972930478074,-0.000235647756792349,3.80197801358169e-05,0.000321661726173334,0.000543595791653537,0.000644104835925005,0.000588844945912514,0.000381973784588710,6.49334891517613e-05,-0.000288951840645289,-0.000593492321711194,-0.000768131381123457,-0.000761179565529975,-0.000562283100847654,-0.000210083087778465,0.000217061369202846,0.000615835391598136,0.000884953858656829,0.000948313052515260,0.000778164443560065,0.000402617249195520,-9.57721550088415e-05,-0.000599851214765325,-0.000982925059372773,-0.00114242756498996,-0.00102480015587528,-0.000644283517924812,-8.07994050962511e-05,0.000534226190937332,0.00105026034065052,0.00133223670076952,0.00129735552689972,0.000936422436097352,0.000322183468886171,-0.000406447531171399,-0.00107300150830799,-0.00150778620717139,-0.00158898492169210,-0.00127789359379036,-0.000631676635553281,0.000205480435592108,0.00103578588223087,0.00165203444124591,0.00188853225913880,0.00166463397417807,0.00101556569271055,8.07346505198323e-05,-0.000924986301950368,-0.00174897886080622,-0.00218106334318668,-0.00209104798819840,-0.00147560705208410,-0.000461466156592793,0.000719168292560208,0.00178015334256871,0.00245160754544550,0.00254612673517478,0.00201148722794215,0.000948917484967274,-0.000403203461952063,-0.00172315033598271,-0.00268014785762408,-0.00301808363129534,-0.00262130441133404,-0.00155212468821863,-4.24245814635350e-05,0.00155380296687732,0.00284398455796397,0.00349112151810480,0.00330196592411329,0.00228075576371429,0.000639685555779194,-0.00124494075125038,-0.00291615727095717,-0.00394736724813684,-0.00404839890491124,-0.00314619916775268,-0.00141269996253721,0.000763817543823744,0.00286531730560341,0.00436434760724723,0.00485590227040089,0.00416296073858980,0.00239414379788477,-6.92633871522469e-05,-0.00265169080000238,-0.00471677657681364,-0.00572072307742373,-0.00535361761996745,-0.00362800246910812,-0.000895334632684916,0.00222293200660433,0.00497204473913865,0.00664450212322939,0.00675694317361097,0.00518482450401190,0.00221406259836600,-0.00150344842508819,-0.00508916383265880,-0.00763832593122044,-0.00844434474260682,-0.00718474600611743,-0.00402945969193484,0.000370694735887264,0.00500716542551709,0.00873707931656989,0.0105569693922753,0.00985833280468664,0.00660729642869476,0.00140348734599666,-0.00462116475466275,-0.0100287408975305,-0.0134041680526106,-0.0137001391812507,-0.0105285523000909,-0.00431738279400548,0.00370978630578411,0.0117453379754635,0.0177664824921516,0.0199936468732958,0.0173392600079425,0.00974957079401820,-0.00164793135265522,-0.0146377023592479,-0.0262522856574784,-0.0332982883404800,-0.0329785813529786,-0.0234889025494915,-0.00446795022720651,0.0227987111313552,0.0554733935803309,0.0895574698627017,0.120515653986008,0.144023869951959,0.156703240541716,0.156703240541716,0.144023869951959,0.120515653986008,0.0895574698627017,0.0554733935803309,0.0227987111313552,-0.00446795022720651,-0.0234889025494915,-0.0329785813529786,-0.0332982883404800,-0.0262522856574784,-0.0146377023592479,-0.00164793135265522,0.00974957079401820,0.0173392600079425,0.0199936468732958,0.0177664824921516,0.0117453379754635,0.00370978630578411,-0.00431738279400548,-0.0105285523000909,-0.0137001391812507,-0.0134041680526106,-0.0100287408975305,-0.00462116475466275,0.00140348734599666,0.00660729642869476,0.00985833280468664,0.0105569693922753,0.00873707931656989,0.00500716542551709,0.000370694735887264,-0.00402945969193484,-0.00718474600611743,-0.00844434474260682,-0.00763832593122044,-0.00508916383265880,-0.00150344842508819,0.00221406259836600,0.00518482450401190,0.00675694317361097,0.00664450212322939,0.00497204473913865,0.00222293200660433,-0.000895334632684916,-0.00362800246910812,-0.00535361761996745,-0.00572072307742373,-0.00471677657681364,-0.00265169080000238,-6.92633871522469e-05,0.00239414379788477,0.00416296073858980,0.00485590227040089,0.00436434760724723,0.00286531730560341,0.000763817543823744,-0.00141269996253721,-0.00314619916775268,-0.00404839890491124,-0.00394736724813684,-0.00291615727095717,-0.00124494075125038,0.000639685555779194,0.00228075576371429,0.00330196592411329,0.00349112151810480,0.00284398455796397,0.00155380296687732,-4.24245814635350e-05,-0.00155212468821863,-0.00262130441133404,-0.00301808363129534,-0.00268014785762408,-0.00172315033598271,-0.000403203461952063,0.000948917484967274,0.00201148722794215,0.00254612673517478,0.00245160754544550,0.00178015334256871,0.000719168292560208,-0.000461466156592793,-0.00147560705208410,-0.00209104798819840,-0.00218106334318668,-0.00174897886080622,-0.000924986301950368,8.07346505198323e-05,0.00101556569271055,0.00166463397417807,0.00188853225913880,0.00165203444124591,0.00103578588223087,0.000205480435592108,-0.000631676635553281,-0.00127789359379036,-0.00158898492169210,-0.00150778620717139,-0.00107300150830799,-0.000406447531171399,0.000322183468886171,0.000936422436097352,0.00129735552689972,0.00133223670076952,0.00105026034065052,0.000534226190937332,-8.07994050962511e-05,-0.000644283517924812,-0.00102480015587528,-0.00114242756498996,-0.000982925059372773,-0.000599851214765325,-9.57721550088415e-05,0.000402617249195520,0.000778164443560065,0.000948313052515260,0.000884953858656829,0.000615835391598136,0.000217061369202846,-0.000210083087778465,-0.000562283100847654,-0.000761179565529975,-0.000768131381123457,-0.000593492321711194,-0.000288951840645289,6.49334891517613e-05,0.000381973784588710,0.000588844945912514,0.000644104835925005,0.000543595791653537,0.000321661726173334,3.80197801358169e-05,-0.000235647756792349,-0.000435972930478074,-0.000519910225057264,-0.000476023611671503,-0.000322747046191756,-0.000104469482676730,0.000123551936096403,0.000306018107911122,0.000403778285772824,0.000399146152793842,0.000301322189111397,0.000139638083216722,-4.23373398720898e-05,-0.000203831166464738,-0.000227768927611145,-0.00207925902605286,-8.45011304875402e-05};
//float aaFilterBuffer[FIR_LENGTH] = {0};
//float usFilterBuffer[FIR_LENGTH/RESAMPLE_RATIO] = {0};
//float firBuffer1[FIR_LENGTH] = {0};
//float iirB[4][3] = {{5.017845e-03,1.017432e-02,5.158427e-03},{5.017845e-03,1.003328e-02,5.017352e-03},{5.017845e-03,9.897058e-03,4.881095e-03},{5.017845e-03,1.003810e-02,5.018338e-03}}; //2k 3db cheby1
//float iirA[4][3] = {{1.000000e+00,2.027628e+00,1.028016e+00},{1.000000e+00,1.999520e+00,9.999017e-01},{1.000000e+00,1.972372e+00,9.727473e-01},{1.000000e+00,2.000480e+00,1.000098e+00}};
//float iirB[4][3] = {{5.107422e-02,1.037041e-01,5.265389e-02},{5.107422e-02,1.021307e-01,5.108017e-02},{5.107422e-02,1.005927e-01,4.954194e-02},{5.107422e-02,1.021662e-01,5.106828e-02}};
//float iirA[4][3] = {{1.000000e+00,-1.162144e+00,3.419283e-01},{1.000000e+00,-1.223429e+00,4.126940e-01},{1.000000e+00,-1.355510e+00,5.652084e-01},{1.000000e+00,-1.578113e+00,8.222485e-01}};
//float iirBuffer[IIR_STAGES * 3] = {0}; //for DIR2
//float iirB[3] = {0.0144,0.0288,0.0144};
//float iirA[3] = {1.0000,1.6330,0.6906};
//float iirBuffer[2][3] = {0};
//float iirB[IIR_STAGES * 3] = {1.464661e-02,2.973936e-02,1.509962e-02,1.464661e-02,2.928814e-02,1.464832e-02,1.464661e-02,2.884710e-02,1.420720e-02,1.464661e-02,2.929832e-02,1.464491e-02}; //2k/48k butter
//float iirA[IIR_STAGES * 3] = {1.000000e+00,-1.540741e+00,5.950923e-01,1.000000e+00,-1.589739e+00,6.458194e-01,1.000000e+00,-1.688988e+00,7.485694e-01,1.000000e+00,-1.838995e+00,9.038678e-01};
//float iirBuffer[(IIR_STAGES + 1) * 3] = {0}; //for DIR1
//int iirCounter = 0;
//float iirB[2] = {0.0065,0.0065};
//float iirA[2] = {1.0000,-0.9870};
//float iirBuffer[4] = {0};
//float iirB[IIR_STAGES * 3] = {3.102688e-01,6.300397e-01,3.199181e-01,3.102688e-01,6.204296e-01,3.103063e-01,3.102688e-01,6.110356e-01,3.009106e-01,3.102688e-01,6.206457e-01,3.102313e-01}; //2k/4k butter
//float iirA[IIR_STAGES * 3] = {1.000000e+00,1.942890e-16,6.735137e-01,1.000000e+00,-1.332268e-15,2.857022e-01,1.000000e+00,-7.771561e-16,9.201921e-02,1.000000e+00,-1.723678e-16,9.700557e-03};
//float iirBuffer[(IIR_STAGES + 1) * 3] = {0}; //for DIR1
//int resampleCnt = 0;
//int aaCnt = 0;
//int usCnt = 0;
//
//float inputBuffer[FFT_LENGTH] = {0};
//float outputBuffer[FFT_LENGTH] = {0};
//int fftFilterCnt = 0;
//
//float i_input[512];
//complex_float r_output[FFT_LENGTH]; /* FFT of input signal */
//complex_float i_output[FFT_LENGTH]; /* inverse of r_output */
//complex_float i_temp[FFT_LENGTH];
//complex_float c_temp[FFT_LENGTH];
//float *r_temp = (float *) c_temp;
//complex_float pm twiddle_table[FFT_LENGTH/2];
//
//complex_float fftFilterCoeff[FFT_LENGTH] = {{1.000000e+00,0.000000e+00},{6.139596e-03,-1.000586e+00},{-1.001343e+00,-1.228895e-02},{-1.842551e-02,1.000850e+00},{9.997403e-01,2.454225e-02},{3.068326e-02,-9.998049e-01},{-1.000499e+00,-3.685055e-02},{-4.297750e-02,9.999908e-01},{1.000290e+00,4.914108e-02},{5.536318e-02,-1.001514e+00},{-9.818870e-01,-6.032355e-02},{-6.043514e-02,8.940396e-01},{7.048820e-01,5.199522e-02},{3.563517e-02,-4.457929e-01},{-2.051621e-01,-1.766750e-02},{-5.264928e-03,5.704171e-02},{4.388725e-03,4.322517e-04},{-1.412973e-04,1.349664e-03},{6.379903e-04,7.075179e-05},{1.574796e-04,-1.344674e-03},{3.346799e-04,4.127883e-05},{2.131249e-04,-1.644834e-03},{2.508867e-04,3.407450e-05},{2.248135e-04,-1.582406e-03},{2.051729e-04,3.043453e-05},{2.254030e-04,-1.457856e-03},{1.728220e-04,2.780730e-05},{2.229178e-04,-1.333221e-03},{1.478734e-04,2.565847e-05},{2.197295e-04,-1.221779e-03},{1.279546e-04,2.382328e-05},{2.165961e-04,-1.124933e-03},{1.117407e-04,2.222661e-05},{2.137499e-04,-1.041164e-03},{9.835735e-05,2.082241e-05},{2.112396e-04,-9.684516e-04},{8.718296e-05,1.957759e-05},{2.090500e-04,-9.049321e-04},{7.775919e-05,1.846660e-05},{2.071458e-04,-8.490419e-04},{6.974045e-05,1.746907e-05},{2.054884e-04,-7.995097e-04},{6.286195e-05,1.656850e-05},{2.040417e-04,-7.553112e-04},{5.691810e-05,1.575136e-05},{2.027743e-04,-7.156208e-04},{5.174740e-05,1.500647e-05},{2.016595e-04,-6.797698e-04},{4.722161e-05,1.432452e-05},{2.006747e-04,-6.472129e-04},{4.323795e-05,1.369772e-05},{1.998012e-04,-6.175022e-04},{3.971325e-05,1.311948e-05},{1.990232e-04,-5.902669e-04},{3.657967e-05,1.258423e-05},{1.983275e-04,-5.651979e-04},{3.378145e-05,1.208720e-05},{1.977033e-04,-5.420352e-04},{3.127241e-05,1.162428e-05},{1.971411e-04,-5.205592e-04},{2.901406e-05,1.119196e-05},{1.966331e-04,-5.005828e-04},{2.697410e-05,1.078716e-05},{1.961726e-04,-4.819457e-04},{2.512527e-05,1.040723e-05},{1.957540e-04,-4.645097e-04},{2.344443e-05,1.004981e-05},{1.953723e-04,-4.481553e-04},{2.191188e-05,9.712866e-06},{1.950234e-04,-4.327784e-04},{2.051067e-05,9.394580e-06},{1.947037e-04,-4.182877e-04},{1.922627e-05,9.093347e-06},{1.944099e-04,-4.046033e-04},{1.804606e-05,8.807743e-06},{1.941395e-04,-3.916545e-04},{1.695910e-05,8.536494e-06},{1.938900e-04,-3.793788e-04},{1.595586e-05,8.278464e-06},{1.936593e-04,-3.677206e-04},{1.502800e-05,8.032631e-06},{1.934455e-04,-3.566302e-04},{1.416817e-05,7.798076e-06},{1.932472e-04,-3.460632e-04},{1.336993e-05,7.573971e-06},{1.930628e-04,-3.359797e-04},{1.262756e-05,7.359564e-06},{1.928911e-04,-3.263438e-04},{1.193602e-05,7.154175e-06},{1.927310e-04,-3.171231e-04},{1.129080e-05,6.957185e-06},{1.925814e-04,-3.082882e-04},{1.068789e-05,6.768031e-06},{1.924415e-04,-2.998123e-04},{1.012371e-05,6.586198e-06},{1.923104e-04,-2.916713e-04},{9.595060e-06,6.411214e-06},{1.921875e-04,-2.838430e-04},{9.099047e-06,6.242648e-06},{1.920720e-04,-2.763072e-04},{8.633081e-06,6.080102e-06},{1.919634e-04,-2.690452e-04},{8.194822e-06,5.923212e-06},{1.918613e-04,-2.620402e-04},{7.782155e-06,5.771640e-06},{1.917650e-04,-2.552765e-04},{7.393165e-06,5.625076e-06},{1.916742e-04,-2.487396e-04},{7.026112e-06,5.483232e-06},{1.915884e-04,-2.424162e-04},{6.679417e-06,5.345841e-06},{1.915073e-04,-2.362941e-04},{6.351641e-06,5.212657e-06},{1.914306e-04,-2.303619e-04},{6.041470e-06,5.083449e-06},{1.913580e-04,-2.246090e-04},{5.747705e-06,4.958005e-06},{1.912892e-04,-2.190256e-04},{5.469248e-06,4.836123e-06},{1.912240e-04,-2.136026e-04},{5.205091e-06,4.717620e-06},{1.911620e-04,-2.083314e-04},{4.954312e-06,4.602320e-06},{1.911032e-04,-2.032042e-04},{4.716060e-06,4.490063e-06},{1.910473e-04,-1.982135e-04},{4.489555e-06,4.380695e-06},{1.909941e-04,-1.933524e-04},{4.274074e-06,4.274074e-06},{1.909434e-04,-1.886145e-04},{4.068954e-06,4.170067e-06},{1.908952e-04,-1.839936e-04},{3.873579e-06,4.068547e-06},{1.908493e-04,-1.794841e-04},{3.687381e-06,3.969397e-06},{1.908055e-04,-1.750805e-04},{3.509834e-06,3.872505e-06},{1.907637e-04,-1.707779e-04},{3.340448e-06,3.777766e-06},{1.907239e-04,-1.665715e-04},{3.178772e-06,3.685080e-06},{1.906858e-04,-1.624568e-04},{3.024384e-06,3.594355e-06},{1.906495e-04,-1.584296e-04},{2.876892e-06,3.505503e-06},{1.906147e-04,-1.544859e-04},{2.735931e-06,3.418438e-06},{1.905815e-04,-1.506219e-04},{2.601163e-06,3.333083e-06},{1.905498e-04,-1.468341e-04},{2.472270e-06,3.249361e-06},{1.905194e-04,-1.431191e-04},{2.348957e-06,3.167202e-06},{1.904903e-04,-1.394737e-04},{2.230947e-06,3.086538e-06},{1.904625e-04,-1.358947e-04},{2.117983e-06,3.007304e-06},{1.904358e-04,-1.323795e-04},{2.009821e-06,2.929440e-06},{1.904103e-04,-1.289250e-04},{1.906237e-06,2.852886e-06},{1.903859e-04,-1.255289e-04},{1.807019e-06,2.777588e-06},{1.903625e-04,-1.221885e-04},{1.711967e-06,2.703492e-06},{1.903400e-04,-1.189016e-04},{1.620896e-06,2.630547e-06},{1.903185e-04,-1.156657e-04},{1.533630e-06,2.558707e-06},{1.902979e-04,-1.124789e-04},{1.450005e-06,2.487923e-06},{1.902782e-04,-1.093389e-04},{1.369866e-06,2.418152e-06},{1.902593e-04,-1.062438e-04},{1.293069e-06,2.349352e-06},{1.902412e-04,-1.031918e-04},{1.219477e-06,2.281481e-06},{1.902238e-04,-1.001810e-04},{1.148962e-06,2.214502e-06},{1.902071e-04,-9.720958e-05},{1.081402e-06,2.148376e-06},{1.901912e-04,-9.427601e-05},{1.016684e-06,2.083068e-06},{1.901759e-04,-9.137862e-05},{9.546999e-07,2.018543e-06},{1.901613e-04,-8.851588e-05},{8.953497e-07,1.954768e-06},{1.901473e-04,-8.568630e-05},{8.385378e-07,1.891711e-06},{1.901339e-04,-8.288844e-05},{7.841745e-07,1.829341e-06},{1.901211e-04,-8.012094e-05},{7.321753e-07,1.767628e-06},{1.901088e-04,-7.738246e-05},{6.824604e-07,1.706543e-06},{1.900971e-04,-7.467171e-05},{6.349549e-07,1.646059e-06},{1.900859e-04,-7.198746e-05},{5.895878e-07,1.586148e-06},{1.900752e-04,-6.932850e-05},{5.462924e-07,1.526785e-06},{1.900650e-04,-6.669367e-05},{5.050061e-07,1.467945e-06},{1.900553e-04,-6.408185e-05},{4.656696e-07,1.409602e-06},{1.900460e-04,-6.149195e-05},{4.282271e-07,1.351733e-06},{1.900372e-04,-5.892290e-05},{3.926264e-07,1.294316e-06},{1.900288e-04,-5.637367e-05},{3.588181e-07,1.237327e-06},{1.900209e-04,-5.384326e-05},{3.267560e-07,1.180745e-06},{1.900134e-04,-5.133069e-05},{2.963966e-07,1.124548e-06},{1.900062e-04,-4.883502e-05},{2.676992e-07,1.068715e-06},{1.899995e-04,-4.635531e-05},{2.406257e-07,1.013227e-06},{1.899932e-04,-4.389067e-05},{2.151404e-07,9.580636e-07},{1.899872e-04,-4.144019e-05},{1.912100e-07,9.032050e-07},{1.899816e-04,-3.900302e-05},{1.688035e-07,8.486323e-07},{1.899764e-04,-3.657831e-05},{1.478921e-07,7.943271e-07},{1.899715e-04,-3.416523e-05},{1.284492e-07,7.402709e-07},{1.899670e-04,-3.176295e-05},{1.104501e-07,6.864458e-07},{1.899628e-04,-2.937067e-05},{9.387207e-08,6.328341e-07},{1.899590e-04,-2.698760e-05},{7.869444e-08,5.794183e-07},{1.899555e-04,-2.461297e-05},{6.489828e-08,5.261814e-07},{1.899523e-04,-2.224600e-05},{5.246649e-08,4.731062e-07},{1.899495e-04,-1.988594e-05},{4.138375e-08,4.201762e-07},{1.899470e-04,-1.753204e-05},{3.163642e-08,3.673748e-07},{1.899448e-04,-1.518354e-05},{2.321259e-08,3.146854e-07},{1.899429e-04,-1.283973e-05},{1.610197e-08,2.620920e-07},{1.899414e-04,-1.049987e-05},{1.029592e-08,2.095782e-07},{1.899401e-04,-8.163227e-06},{5.787369e-09,1.571281e-07},{1.899392e-04,-5.829091e-06},{2.570869e-09,1.047256e-07},{1.899386e-04,-3.496741e-06},{6.425230e-10,5.235486e-08},{1.899383e-04,-1.165461e-06},{0.000000e+00,0.000000e+00},{1.899383e-04,1.165461e-06},{6.425230e-10,-5.235486e-08},{1.899386e-04,3.496741e-06},{2.570869e-09,-1.047256e-07},{1.899392e-04,5.829091e-06},{5.787369e-09,-1.571281e-07},{1.899401e-04,8.163227e-06},{1.029592e-08,-2.095782e-07},{1.899414e-04,1.049987e-05},{1.610197e-08,-2.620920e-07},{1.899429e-04,1.283973e-05},{2.321259e-08,-3.146854e-07},{1.899448e-04,1.518354e-05},{3.163642e-08,-3.673748e-07},{1.899470e-04,1.753204e-05},{4.138375e-08,-4.201762e-07},{1.899495e-04,1.988594e-05},{5.246649e-08,-4.731062e-07},{1.899523e-04,2.224600e-05},{6.489828e-08,-5.261814e-07},{1.899555e-04,2.461297e-05},{7.869444e-08,-5.794183e-07},{1.899590e-04,2.698760e-05},{9.387207e-08,-6.328341e-07},{1.899628e-04,2.937067e-05},{1.104501e-07,-6.864458e-07},{1.899670e-04,3.176295e-05},{1.284492e-07,-7.402709e-07},{1.899715e-04,3.416523e-05},{1.478921e-07,-7.943271e-07},{1.899764e-04,3.657831e-05},{1.688035e-07,-8.486323e-07},{1.899816e-04,3.900302e-05},{1.912100e-07,-9.032050e-07},{1.899872e-04,4.144019e-05},{2.151404e-07,-9.580636e-07},{1.899932e-04,4.389067e-05},{2.406257e-07,-1.013227e-06},{1.899995e-04,4.635531e-05},{2.676992e-07,-1.068715e-06},{1.900062e-04,4.883502e-05},{2.963966e-07,-1.124548e-06},{1.900134e-04,5.133069e-05},{3.267560e-07,-1.180745e-06},{1.900209e-04,5.384326e-05},{3.588181e-07,-1.237327e-06},{1.900288e-04,5.637367e-05},{3.926264e-07,-1.294316e-06},{1.900372e-04,5.892290e-05},{4.282271e-07,-1.351733e-06},{1.900460e-04,6.149195e-05},{4.656696e-07,-1.409602e-06},{1.900553e-04,6.408185e-05},{5.050061e-07,-1.467945e-06},{1.900650e-04,6.669367e-05},{5.462924e-07,-1.526785e-06},{1.900752e-04,6.932850e-05},{5.895878e-07,-1.586148e-06},{1.900859e-04,7.198746e-05},{6.349549e-07,-1.646059e-06},{1.900971e-04,7.467171e-05},{6.824604e-07,-1.706543e-06},{1.901088e-04,7.738246e-05},{7.321753e-07,-1.767628e-06},{1.901211e-04,8.012094e-05},{7.841745e-07,-1.829341e-06},{1.901339e-04,8.288844e-05},{8.385378e-07,-1.891711e-06},{1.901473e-04,8.568630e-05},{8.953497e-07,-1.954768e-06},{1.901613e-04,8.851588e-05},{9.546999e-07,-2.018543e-06},{1.901759e-04,9.137862e-05},{1.016684e-06,-2.083068e-06},{1.901912e-04,9.427601e-05},{1.081402e-06,-2.148376e-06},{1.902071e-04,9.720958e-05},{1.148962e-06,-2.214502e-06},{1.902238e-04,1.001810e-04},{1.219477e-06,-2.281481e-06},{1.902412e-04,1.031918e-04},{1.293069e-06,-2.349352e-06},{1.902593e-04,1.062438e-04},{1.369866e-06,-2.418152e-06},{1.902782e-04,1.093389e-04},{1.450005e-06,-2.487923e-06},{1.902979e-04,1.124789e-04},{1.533630e-06,-2.558707e-06},{1.903185e-04,1.156657e-04},{1.620896e-06,-2.630547e-06},{1.903400e-04,1.189016e-04},{1.711967e-06,-2.703492e-06},{1.903625e-04,1.221885e-04},{1.807019e-06,-2.777588e-06},{1.903859e-04,1.255289e-04},{1.906237e-06,-2.852886e-06},{1.904103e-04,1.289250e-04},{2.009821e-06,-2.929440e-06},{1.904358e-04,1.323795e-04},{2.117983e-06,-3.007304e-06},{1.904625e-04,1.358947e-04},{2.230947e-06,-3.086538e-06},{1.904903e-04,1.394737e-04},{2.348957e-06,-3.167202e-06},{1.905194e-04,1.431191e-04},{2.472270e-06,-3.249361e-06},{1.905498e-04,1.468341e-04},{2.601163e-06,-3.333083e-06},{1.905815e-04,1.506219e-04},{2.735931e-06,-3.418438e-06},{1.906147e-04,1.544859e-04},{2.876892e-06,-3.505503e-06},{1.906495e-04,1.584296e-04},{3.024384e-06,-3.594355e-06},{1.906858e-04,1.624568e-04},{3.178772e-06,-3.685080e-06},{1.907239e-04,1.665715e-04},{3.340448e-06,-3.777766e-06},{1.907637e-04,1.707779e-04},{3.509834e-06,-3.872505e-06},{1.908055e-04,1.750805e-04},{3.687381e-06,-3.969397e-06},{1.908493e-04,1.794841e-04},{3.873579e-06,-4.068547e-06},{1.908952e-04,1.839936e-04},{4.068954e-06,-4.170067e-06},{1.909434e-04,1.886145e-04},{4.274074e-06,-4.274074e-06},{1.909941e-04,1.933524e-04},{4.489555e-06,-4.380695e-06},{1.910473e-04,1.982135e-04},{4.716060e-06,-4.490063e-06},{1.911032e-04,2.032042e-04},{4.954312e-06,-4.602320e-06},{1.911620e-04,2.083314e-04},{5.205091e-06,-4.717620e-06},{1.912240e-04,2.136026e-04},{5.469248e-06,-4.836123e-06},{1.912892e-04,2.190256e-04},{5.747705e-06,-4.958005e-06},{1.913580e-04,2.246090e-04},{6.041470e-06,-5.083449e-06},{1.914306e-04,2.303619e-04},{6.351641e-06,-5.212657e-06},{1.915073e-04,2.362941e-04},{6.679417e-06,-5.345841e-06},{1.915884e-04,2.424162e-04},{7.026112e-06,-5.483232e-06},{1.916742e-04,2.487396e-04},{7.393165e-06,-5.625076e-06},{1.917650e-04,2.552765e-04},{7.782155e-06,-5.771640e-06},{1.918613e-04,2.620402e-04},{8.194822e-06,-5.923212e-06},{1.919634e-04,2.690452e-04},{8.633081e-06,-6.080102e-06},{1.920720e-04,2.763072e-04},{9.099047e-06,-6.242648e-06},{1.921875e-04,2.838430e-04},{9.595060e-06,-6.411214e-06},{1.923104e-04,2.916713e-04},{1.012371e-05,-6.586198e-06},{1.924415e-04,2.998123e-04},{1.068789e-05,-6.768031e-06},{1.925814e-04,3.082882e-04},{1.129080e-05,-6.957185e-06},{1.927310e-04,3.171231e-04},{1.193602e-05,-7.154175e-06},{1.928911e-04,3.263438e-04},{1.262756e-05,-7.359564e-06},{1.930628e-04,3.359797e-04},{1.336993e-05,-7.573971e-06},{1.932472e-04,3.460632e-04},{1.416817e-05,-7.798076e-06},{1.934455e-04,3.566302e-04},{1.502800e-05,-8.032631e-06},{1.936593e-04,3.677206e-04},{1.595586e-05,-8.278464e-06},{1.938900e-04,3.793788e-04},{1.695910e-05,-8.536494e-06},{1.941395e-04,3.916545e-04},{1.804606e-05,-8.807743e-06},{1.944099e-04,4.046033e-04},{1.922627e-05,-9.093347e-06},{1.947037e-04,4.182877e-04},{2.051067e-05,-9.394580e-06},{1.950234e-04,4.327784e-04},{2.191188e-05,-9.712866e-06},{1.953723e-04,4.481553e-04},{2.344443e-05,-1.004981e-05},{1.957540e-04,4.645097e-04},{2.512527e-05,-1.040723e-05},{1.961726e-04,4.819457e-04},{2.697410e-05,-1.078716e-05},{1.966331e-04,5.005828e-04},{2.901406e-05,-1.119196e-05},{1.971411e-04,5.205592e-04},{3.127241e-05,-1.162428e-05},{1.977033e-04,5.420352e-04},{3.378145e-05,-1.208720e-05},{1.983275e-04,5.651979e-04},{3.657967e-05,-1.258423e-05},{1.990232e-04,5.902669e-04},{3.971325e-05,-1.311948e-05},{1.998012e-04,6.175022e-04},{4.323795e-05,-1.369772e-05},{2.006747e-04,6.472129e-04},{4.722161e-05,-1.432452e-05},{2.016595e-04,6.797698e-04},{5.174740e-05,-1.500647e-05},{2.027743e-04,7.156208e-04},{5.691810e-05,-1.575136e-05},{2.040417e-04,7.553112e-04},{6.286195e-05,-1.656850e-05},{2.054884e-04,7.995097e-04},{6.974045e-05,-1.746907e-05},{2.071458e-04,8.490419e-04},{7.775919e-05,-1.846660e-05},{2.090500e-04,9.049321e-04},{8.718296e-05,-1.957759e-05},{2.112396e-04,9.684516e-04},{9.835735e-05,-2.082241e-05},{2.137499e-04,1.041164e-03},{1.117407e-04,-2.222661e-05},{2.165961e-04,1.124933e-03},{1.279546e-04,-2.382328e-05},{2.197295e-04,1.221779e-03},{1.478734e-04,-2.565847e-05},{2.229178e-04,1.333221e-03},{1.728220e-04,-2.780730e-05},{2.254030e-04,1.457856e-03},{2.051729e-04,-3.043453e-05},{2.248135e-04,1.582406e-03},{2.508867e-04,-3.407450e-05},{2.131249e-04,1.644834e-03},{3.346799e-04,-4.127883e-05},{1.574796e-04,1.344674e-03},{6.379903e-04,-7.075179e-05},{-1.412973e-04,-1.349664e-03},{4.388725e-03,-4.322517e-04},{-5.264928e-03,-5.704171e-02},{-2.051621e-01,1.766750e-02},{3.563517e-02,4.457929e-01},{7.048820e-01,-5.199522e-02},{-6.043514e-02,-8.940396e-01},{-9.818870e-01,6.032355e-02},{5.536318e-02,1.001514e+00},{1.000290e+00,-4.914108e-02},{-4.297750e-02,-9.999908e-01},{-1.000499e+00,3.685055e-02},{3.068326e-02,9.998049e-01},{9.997403e-01,-2.454225e-02},{-1.842551e-02,-1.000850e+00},{-1.001343e+00,1.228895e-02},{6.139596e-03,1.000586e+00}};
//
//cycle_stats_t stats;
//
//complex_float B1[512] = {{-3.395951e-03,0.000000e+00},{3.225008e-03,2.236001e-04},{-3.465351e-03,-5.532362e-04},{3.362225e-03,7.096815e-04},{-3.692397e-03,-1.204507e-03},{3.675831e-03,1.337288e-03},{-4.147031e-03,-2.117883e-03},{4.273842e-03,2.323890e-03},{-5.007758e-03,-3.706855e-03},{5.427776e-03,4.366458e-03},{-6.697147e-03,-7.585777e-03},{7.318193e-03,1.175144e-02},{-9.186417e-04,-2.464194e-02},{-2.248657e-02,1.833101e-02},{1.936992e-02,2.995751e-03},{-1.016328e-02,-4.389697e-03},{6.672781e-03,5.093068e-03},{-4.603175e-03,-3.391101e-03},{3.610662e-03,3.810680e-03},{-2.770803e-03,-2.643079e-03},{2.322722e-03,3.029275e-03},{-1.883520e-03,-2.169300e-03},{1.628708e-03,2.524878e-03},{-1.368418e-03,-1.846913e-03},{1.200797e-03,2.173222e-03},{-1.035900e-03,-1.613186e-03},{9.136667e-04,1.913277e-03},{-8.057228e-04,-1.435390e-03},{7.094700e-04,1.712553e-03},{-6.383012e-04,-1.295107e-03},{5.579532e-04,1.552321e-03},{-5.119321e-04,-1.181238e-03},{4.418182e-04,1.421046e-03},{-4.137637e-04,-1.086705e-03},{3.504905e-04,1.311240e-03},{-3.357232e-04,-1.006780e-03},{2.771605e-04,1.217825e-03},{-2.724983e-04,-9.381821e-04},{2.172566e-04,1.137229e-03},{-2.204589e-04,-8.785583e-04},{1.676027e-04,1.066867e-03},{-1.770455e-04,-8.261771e-04},{1.259286e-04,1.004817e-03},{-1.404059e-04,-7.797332e-04},{9.057196e-05,9.496167e-04},{-1.091690e-04,-7.382231e-04},{6.028979e-05,9.001369e-04},{-8.230034e-05,-7.008617e-04},{3.413592e-05,8.554872e-04},{-5.900593e-05,-6.670251e-04},{1.137859e-05,8.149568e-04},{-3.866731e-05,-6.362113e-04},{-8.556137e-06,7.779704e-04},{-2.079632e-05,-6.080108e-04},{-2.612402e-05,7.440574e-04},{-5.003100e-06,-5.820864e-04},{-4.169110e-05,7.128283e-04},{9.027080e-06,-5.581576e-04},{-5.555428e-05,6.839582e-04},{2.155072e-05,-5.359890e-04},{-6.795667e-05,6.571733e-04},{3.277871e-05,-5.153815e-04},{-7.909895e-05,6.322416e-04},{4.288584e-05,-4.961654e-04},{-8.914808e-05,6.089648e-04},{5.201804e-05,-4.781951e-04},{-9.824394e-05,5.871722e-04},{6.029795e-05,-4.613451e-04},{-1.065045e-04,5.667165e-04},{6.782935e-05,-4.455062e-04},{-1.140297e-04,5.474692e-04},{7.470047e-05,-4.305835e-04},{-1.209051e-04,5.293182e-04},{8.098683e-05,-4.164935e-04},{-1.272037e-04,5.121650e-04},{8.675331e-05,-4.031627e-04},{-1.329887e-04,4.959226e-04},{9.205597e-05,-3.905263e-04},{-1.383147e-04,4.805142e-04},{9.694344e-05,-3.785264e-04},{-1.432291e-04,4.658712e-04},{1.014581e-04,-3.671118e-04},{-1.477734e-04,4.519325e-04},{1.056369e-04,-3.562365e-04},{-1.519839e-04,4.386434e-04},{1.095125e-04,-3.458591e-04},{-1.558924e-04,4.259549e-04},{1.131134e-04,-3.359427e-04},{-1.595272e-04,4.138225e-04},{1.164650e-04,-3.264537e-04},{-1.629131e-04,4.022063e-04},{1.195897e-04,-3.173618e-04},{-1.660723e-04,3.910701e-04},{1.225074e-04,-3.086395e-04},{-1.690245e-04,3.803809e-04},{1.252359e-04,-3.002618e-04},{-1.717872e-04,3.701087e-04},{1.277911e-04,-2.922059e-04},{-1.743763e-04,3.602262e-04},{1.301872e-04,-2.844509e-04},{-1.768057e-04,3.507084e-04},{1.324371e-04,-2.769777e-04},{-1.790883e-04,3.415323e-04},{1.345523e-04,-2.697688e-04},{-1.812355e-04,3.326768e-04},{1.365431e-04,-2.628082e-04},{-1.832576e-04,3.241227e-04},{1.384190e-04,-2.560809e-04},{-1.851640e-04,3.158519e-04},{1.401884e-04,-2.495733e-04},{-1.869631e-04,3.078480e-04},{1.418592e-04,-2.432727e-04},{-1.886627e-04,3.000959e-04},{1.434383e-04,-2.371674e-04},{-1.902699e-04,2.925812e-04},{1.449322e-04,-2.312465e-04},{-1.917910e-04,2.852909e-04},{1.463467e-04,-2.255000e-04},{-1.932318e-04,2.782128e-04},{1.476871e-04,-2.199185e-04},{-1.945978e-04,2.713356e-04},{1.489584e-04,-2.144932e-04},{-1.958939e-04,2.646487e-04},{1.501651e-04,-2.092159e-04},{-1.971245e-04,2.581421e-04},{1.513113e-04,-2.040791e-04},{-1.982938e-04,2.518068e-04},{1.524007e-04,-1.990757e-04},{-1.994057e-04,2.456340e-04},{1.534370e-04,-1.941988e-04},{-2.004635e-04,2.396157e-04},{1.544232e-04,-1.894424e-04},{-2.014707e-04,2.337442e-04},{1.553624e-04,-1.848006e-04},{-2.024301e-04,2.280126e-04},{1.562574e-04,-1.802677e-04},{-2.033446e-04,2.224140e-04},{1.571107e-04,-1.758387e-04},{-2.042167e-04,2.169422e-04},{1.579246e-04,-1.715087e-04},{-2.050487e-04,2.115912e-04},{1.587014e-04,-1.672730e-04},{-2.058430e-04,2.063555e-04},{1.594431e-04,-1.631273e-04},{-2.066015e-04,2.012298e-04},{1.601515e-04,-1.590675e-04},{-2.073262e-04,1.962090e-04},{1.608285e-04,-1.550897e-04},{-2.080189e-04,1.912884e-04},{1.614756e-04,-1.511903e-04},{-2.086812e-04,1.864637e-04},{1.620945e-04,-1.473658e-04},{-2.093146e-04,1.817304e-04},{1.626865e-04,-1.436128e-04},{-2.099206e-04,1.770847e-04},{1.632529e-04,-1.399283e-04},{-2.105005e-04,1.725227e-04},{1.637951e-04,-1.363093e-04},{-2.110556e-04,1.680407e-04},{1.643141e-04,-1.327529e-04},{-2.115871e-04,1.636354e-04},{1.648111e-04,-1.292564e-04},{-2.120961e-04,1.593034e-04},{1.652871e-04,-1.258173e-04},{-2.125836e-04,1.550416e-04},{1.657430e-04,-1.224332e-04},{-2.130506e-04,1.508470e-04},{1.661798e-04,-1.191017e-04},{-2.134980e-04,1.467167e-04},{1.665983e-04,-1.158205e-04},{-2.139267e-04,1.426481e-04},{1.669992e-04,-1.125876e-04},{-2.143374e-04,1.386385e-04},{1.673833e-04,-1.094010e-04},{-2.147309e-04,1.346855e-04},{1.677514e-04,-1.062586e-04},{-2.151079e-04,1.307867e-04},{1.681040e-04,-1.031586e-04},{-2.154691e-04,1.269397e-04},{1.684418e-04,-1.000992e-04},{-2.158151e-04,1.231423e-04},{1.687654e-04,-9.707865e-05},{-2.161465e-04,1.193926e-04},{1.690753e-04,-9.409538e-05},{-2.164638e-04,1.156884e-04},{1.693720e-04,-9.114776e-05},{-2.167677e-04,1.120278e-04},{1.696560e-04,-8.823426e-05},{-2.170585e-04,1.084089e-04},{1.699278e-04,-8.535341e-05},{-2.173367e-04,1.048299e-04},{1.701878e-04,-8.250379e-05},{-2.176028e-04,1.012891e-04},{1.704365e-04,-7.968403e-05},{-2.178572e-04,9.778483e-05},{1.706741e-04,-7.689282e-05},{-2.181002e-04,9.431539e-05},{1.709010e-04,-7.412887e-05},{-2.183323e-04,9.087926e-05},{1.711177e-04,-7.139095e-05},{-2.185537e-04,8.747491e-05},{1.713243e-04,-6.867786e-05},{-2.187649e-04,8.410087e-05},{1.715213e-04,-6.598845e-05},{-2.189661e-04,8.075571e-05},{1.717088e-04,-6.332159e-05},{-2.191575e-04,7.743805e-05},{1.718873e-04,-6.067620e-05},{-2.193396e-04,7.414655e-05},{1.720568e-04,-5.805120e-05},{-2.195124e-04,7.087989e-05},{1.722177e-04,-5.544557e-05},{-2.196763e-04,6.763681e-05},{1.723701e-04,-5.285831e-05},{-2.198315e-04,6.441606e-05},{1.725143e-04,-5.028843e-05},{-2.199782e-04,6.121644e-05},{1.726505e-04,-4.773498e-05},{-2.201166e-04,5.803677e-05},{1.727788e-04,-4.519703e-05},{-2.202469e-04,5.487590e-05},{1.728995e-04,-4.267366e-05},{-2.203692e-04,5.173269e-05},{1.730126e-04,-4.016399e-05},{-2.204837e-04,4.860605e-05},{1.731183e-04,-3.766714e-05},{-2.205905e-04,4.549489e-05},{1.732169e-04,-3.518226e-05},{-2.206899e-04,4.239816e-05},{1.733082e-04,-3.270849e-05},{-2.207819e-04,3.931481e-05},{1.733926e-04,-3.024502e-05},{-2.208666e-04,3.624381e-05},{1.734702e-04,-2.779104e-05},{-2.209441e-04,3.318416e-05},{1.735409e-04,-2.534573e-05},{-2.210146e-04,3.013486e-05},{1.736049e-04,-2.290830e-05},{-2.210781e-04,2.709492e-05},{1.736623e-04,-2.047799e-05},{-2.211348e-04,2.406339e-05},{1.737131e-04,-1.805401e-05},{-2.211846e-04,2.103929e-05},{1.737575e-04,-1.563560e-05},{-2.212276e-04,1.802169e-05},{1.737954e-04,-1.322201e-05},{-2.212639e-04,1.500963e-05},{1.738269e-04,-1.081249e-05},{-2.212936e-04,1.200218e-05},{1.738521e-04,-8.406278e-06},{-2.213166e-04,8.998421e-06},{1.738709e-04,-6.002646e-06},{-2.213330e-04,5.997417e-06},{1.738835e-04,-3.600853e-06},{-2.213429e-04,2.998249e-06},{1.738897e-04,-1.200162e-06},{-2.213462e-04,0.000000e+00},{1.738897e-04,1.200162e-06},{-2.213429e-04,-2.998249e-06},{1.738835e-04,3.600853e-06},{-2.213330e-04,-5.997417e-06},{1.738709e-04,6.002646e-06},{-2.213166e-04,-8.998421e-06},{1.738521e-04,8.406278e-06},{-2.212936e-04,-1.200218e-05},{1.738269e-04,1.081249e-05},{-2.212639e-04,-1.500963e-05},{1.737954e-04,1.322201e-05},{-2.212276e-04,-1.802169e-05},{1.737575e-04,1.563560e-05},{-2.211846e-04,-2.103929e-05},{1.737131e-04,1.805401e-05},{-2.211348e-04,-2.406339e-05},{1.736623e-04,2.047799e-05},{-2.210781e-04,-2.709492e-05},{1.736049e-04,2.290830e-05},{-2.210146e-04,-3.013486e-05},{1.735409e-04,2.534573e-05},{-2.209441e-04,-3.318416e-05},{1.734702e-04,2.779104e-05},{-2.208666e-04,-3.624381e-05},{1.733926e-04,3.024502e-05},{-2.207819e-04,-3.931481e-05},{1.733082e-04,3.270849e-05},{-2.206899e-04,-4.239816e-05},{1.732169e-04,3.518226e-05},{-2.205905e-04,-4.549489e-05},{1.731183e-04,3.766714e-05},{-2.204837e-04,-4.860605e-05},{1.730126e-04,4.016399e-05},{-2.203692e-04,-5.173269e-05},{1.728995e-04,4.267366e-05},{-2.202469e-04,-5.487590e-05},{1.727788e-04,4.519703e-05},{-2.201166e-04,-5.803677e-05},{1.726505e-04,4.773498e-05},{-2.199782e-04,-6.121644e-05},{1.725143e-04,5.028843e-05},{-2.198315e-04,-6.441606e-05},{1.723701e-04,5.285831e-05},{-2.196763e-04,-6.763681e-05},{1.722177e-04,5.544557e-05},{-2.195124e-04,-7.087989e-05},{1.720568e-04,5.805120e-05},{-2.193396e-04,-7.414655e-05},{1.718873e-04,6.067620e-05},{-2.191575e-04,-7.743805e-05},{1.717088e-04,6.332159e-05},{-2.189661e-04,-8.075571e-05},{1.715213e-04,6.598845e-05},{-2.187649e-04,-8.410087e-05},{1.713243e-04,6.867786e-05},{-2.185537e-04,-8.747491e-05},{1.711177e-04,7.139095e-05},{-2.183323e-04,-9.087926e-05},{1.709010e-04,7.412887e-05},{-2.181002e-04,-9.431539e-05},{1.706741e-04,7.689282e-05},{-2.178572e-04,-9.778483e-05},{1.704365e-04,7.968403e-05},{-2.176028e-04,-1.012891e-04},{1.701878e-04,8.250379e-05},{-2.173367e-04,-1.048299e-04},{1.699278e-04,8.535341e-05},{-2.170585e-04,-1.084089e-04},{1.696560e-04,8.823426e-05},{-2.167677e-04,-1.120278e-04},{1.693720e-04,9.114776e-05},{-2.164638e-04,-1.156884e-04},{1.690753e-04,9.409538e-05},{-2.161465e-04,-1.193926e-04},{1.687654e-04,9.707865e-05},{-2.158151e-04,-1.231423e-04},{1.684418e-04,1.000992e-04},{-2.154691e-04,-1.269397e-04},{1.681040e-04,1.031586e-04},{-2.151079e-04,-1.307867e-04},{1.677514e-04,1.062586e-04},{-2.147309e-04,-1.346855e-04},{1.673833e-04,1.094010e-04},{-2.143374e-04,-1.386385e-04},{1.669992e-04,1.125876e-04},{-2.139267e-04,-1.426481e-04},{1.665983e-04,1.158205e-04},{-2.134980e-04,-1.467167e-04},{1.661798e-04,1.191017e-04},{-2.130506e-04,-1.508470e-04},{1.657430e-04,1.224332e-04},{-2.125836e-04,-1.550416e-04},{1.652871e-04,1.258173e-04},{-2.120961e-04,-1.593034e-04},{1.648111e-04,1.292564e-04},{-2.115871e-04,-1.636354e-04},{1.643141e-04,1.327529e-04},{-2.110556e-04,-1.680407e-04},{1.637951e-04,1.363093e-04},{-2.105005e-04,-1.725227e-04},{1.632529e-04,1.399283e-04},{-2.099206e-04,-1.770847e-04},{1.626865e-04,1.436128e-04},{-2.093146e-04,-1.817304e-04},{1.620945e-04,1.473658e-04},{-2.086812e-04,-1.864637e-04},{1.614756e-04,1.511903e-04},{-2.080189e-04,-1.912884e-04},{1.608285e-04,1.550897e-04},{-2.073262e-04,-1.962090e-04},{1.601515e-04,1.590675e-04},{-2.066015e-04,-2.012298e-04},{1.594431e-04,1.631273e-04},{-2.058430e-04,-2.063555e-04},{1.587014e-04,1.672730e-04},{-2.050487e-04,-2.115912e-04},{1.579246e-04,1.715087e-04},{-2.042167e-04,-2.169422e-04},{1.571107e-04,1.758387e-04},{-2.033446e-04,-2.224140e-04},{1.562574e-04,1.802677e-04},{-2.024301e-04,-2.280126e-04},{1.553624e-04,1.848006e-04},{-2.014707e-04,-2.337442e-04},{1.544232e-04,1.894424e-04},{-2.004635e-04,-2.396157e-04},{1.534370e-04,1.941988e-04},{-1.994057e-04,-2.456340e-04},{1.524007e-04,1.990757e-04},{-1.982938e-04,-2.518068e-04},{1.513113e-04,2.040791e-04},{-1.971245e-04,-2.581421e-04},{1.501651e-04,2.092159e-04},{-1.958939e-04,-2.646487e-04},{1.489584e-04,2.144932e-04},{-1.945978e-04,-2.713356e-04},{1.476871e-04,2.199185e-04},{-1.932318e-04,-2.782128e-04},{1.463467e-04,2.255000e-04},{-1.917910e-04,-2.852909e-04},{1.449322e-04,2.312465e-04},{-1.902699e-04,-2.925812e-04},{1.434383e-04,2.371674e-04},{-1.886627e-04,-3.000959e-04},{1.418592e-04,2.432727e-04},{-1.869631e-04,-3.078480e-04},{1.401884e-04,2.495733e-04},{-1.851640e-04,-3.158519e-04},{1.384190e-04,2.560809e-04},{-1.832576e-04,-3.241227e-04},{1.365431e-04,2.628082e-04},{-1.812355e-04,-3.326768e-04},{1.345523e-04,2.697688e-04},{-1.790883e-04,-3.415323e-04},{1.324371e-04,2.769777e-04},{-1.768057e-04,-3.507084e-04},{1.301872e-04,2.844509e-04},{-1.743763e-04,-3.602262e-04},{1.277911e-04,2.922059e-04},{-1.717872e-04,-3.701087e-04},{1.252359e-04,3.002618e-04},{-1.690245e-04,-3.803809e-04},{1.225074e-04,3.086395e-04},{-1.660723e-04,-3.910701e-04},{1.195897e-04,3.173618e-04},{-1.629131e-04,-4.022063e-04},{1.164650e-04,3.264537e-04},{-1.595272e-04,-4.138225e-04},{1.131134e-04,3.359427e-04},{-1.558924e-04,-4.259549e-04},{1.095125e-04,3.458591e-04},{-1.519839e-04,-4.386434e-04},{1.056369e-04,3.562365e-04},{-1.477734e-04,-4.519325e-04},{1.014581e-04,3.671118e-04},{-1.432291e-04,-4.658712e-04},{9.694344e-05,3.785264e-04},{-1.383147e-04,-4.805142e-04},{9.205597e-05,3.905263e-04},{-1.329887e-04,-4.959226e-04},{8.675331e-05,4.031627e-04},{-1.272037e-04,-5.121650e-04},{8.098683e-05,4.164935e-04},{-1.209051e-04,-5.293182e-04},{7.470047e-05,4.305835e-04},{-1.140297e-04,-5.474692e-04},{6.782935e-05,4.455062e-04},{-1.065045e-04,-5.667165e-04},{6.029795e-05,4.613451e-04},{-9.824394e-05,-5.871722e-04},{5.201804e-05,4.781951e-04},{-8.914808e-05,-6.089648e-04},{4.288584e-05,4.961654e-04},{-7.909895e-05,-6.322416e-04},{3.277871e-05,5.153815e-04},{-6.795667e-05,-6.571733e-04},{2.155072e-05,5.359890e-04},{-5.555428e-05,-6.839582e-04},{9.027080e-06,5.581576e-04},{-4.169110e-05,-7.128283e-04},{-5.003100e-06,5.820864e-04},{-2.612402e-05,-7.440574e-04},{-2.079632e-05,6.080108e-04},{-8.556137e-06,-7.779704e-04},{-3.866731e-05,6.362113e-04},{1.137859e-05,-8.149568e-04},{-5.900593e-05,6.670251e-04},{3.413592e-05,-8.554872e-04},{-8.230034e-05,7.008617e-04},{6.028979e-05,-9.001369e-04},{-1.091690e-04,7.382231e-04},{9.057196e-05,-9.496167e-04},{-1.404059e-04,7.797332e-04},{1.259286e-04,-1.004817e-03},{-1.770455e-04,8.261771e-04},{1.676027e-04,-1.066867e-03},{-2.204589e-04,8.785583e-04},{2.172566e-04,-1.137229e-03},{-2.724983e-04,9.381821e-04},{2.771605e-04,-1.217825e-03},{-3.357232e-04,1.006780e-03},{3.504905e-04,-1.311240e-03},{-4.137637e-04,1.086705e-03},{4.418182e-04,-1.421046e-03},{-5.119321e-04,1.181238e-03},{5.579532e-04,-1.552321e-03},{-6.383012e-04,1.295107e-03},{7.094700e-04,-1.712553e-03},{-8.057228e-04,1.435390e-03},{9.136667e-04,-1.913277e-03},{-1.035900e-03,1.613186e-03},{1.200797e-03,-2.173222e-03},{-1.368418e-03,1.846913e-03},{1.628708e-03,-2.524878e-03},{-1.883520e-03,2.169300e-03},{2.322722e-03,-3.029275e-03},{-2.770803e-03,2.643079e-03},{3.610662e-03,-3.810680e-03},{-4.603175e-03,3.391101e-03},{6.672781e-03,-5.093068e-03},{-1.016328e-02,4.389697e-03},{1.936992e-02,-2.995751e-03},{-2.248657e-02,-1.833101e-02},{-9.186417e-04,2.464194e-02},{7.318193e-03,-1.175144e-02},{-6.697147e-03,7.585777e-03},{5.427776e-03,-4.366458e-03},{-5.007758e-03,3.706855e-03},{4.273842e-03,-2.323890e-03},{-4.147031e-03,2.117883e-03},{3.675831e-03,-1.337288e-03},{-3.692397e-03,1.204507e-03},{3.362225e-03,-7.096815e-04},{-3.465351e-03,5.532362e-04},{3.225008e-03,-2.236001e-04}};
//complex_float B2[512] = {{5.033960e-01,0.000000e+00},{-4.966120e-01,-2.778743e-02},{5.028096e-01,5.688214e-02},{-4.951476e-01,-8.457449e-02},{5.010047e-01,1.164641e-01},{-4.920420e-01,-1.453929e-01},{4.978219e-01,1.824441e-01},{-4.868308e-01,-2.146209e-01},{4.928898e-01,2.616729e-01},{-4.783659e-01,-3.013647e-01},{4.851274e-01,3.723914e-01},{-4.634791e-01,-4.347373e-01},{4.555769e-01,6.293127e-01},{-1.079679e-01,-7.509862e-01},{-6.292887e-02,4.970809e-01},{2.704656e-02,-4.094653e-01},{-4.097981e-02,3.448773e-01},{2.789522e-02,-3.150421e-01},{-3.485256e-02,2.788376e-01},{2.754295e-02,-2.622079e-01},{-3.192231e-02,2.375351e-01},{2.715000e-02,-2.267108e-01},{-3.020323e-02,2.082545e-01},{2.682713e-02,-2.006342e-01},{-2.907877e-02,1.860415e-01},{2.657263e-02,-1.804162e-01},{-2.829116e-02,1.684429e-01},{2.637185e-02,-1.641575e-01},{-2.771266e-02,1.540680e-01},{2.621171e-02,-1.507303e-01},{-2.727255e-02,1.420551e-01},{2.608229e-02,-1.394138e-01},{-2.692848e-02,1.318355e-01},{2.597635e-02,-1.297208e-01},{-2.665356e-02,1.230155e-01},{2.588858e-02,-1.213084e-01},{-2.642993e-02,1.153126e-01},{2.581508e-02,-1.139267e-01},{-2.624525e-02,1.085175e-01},{2.575291e-02,-1.073887e-01},{-2.609077e-02,1.024716e-01},{2.569987e-02,-1.015512e-01},{-2.596012e-02,9.705222e-02},{2.565424e-02,-9.630258e-02},{-2.584854e-02,9.216259e-02},{2.561471e-02,-9.155408e-02},{-2.575243e-02,8.772535e-02},{2.558023e-02,-8.723443e-02},{-2.566900e-02,8.367780e-02},{2.554998e-02,-8.328550e-02},{-2.559610e-02,7.996854e-02},{2.552329e-02,-7.965942e-02},{-2.553200e-02,7.655495e-02},{2.549962e-02,-7.631634e-02},{-2.547531e-02,7.340143e-02},{2.547853e-02,-7.322283e-02},{-2.542494e-02,7.047793e-02},{2.545967e-02,-7.035059e-02},{-2.537995e-02,6.775894e-02},{2.544272e-02,-6.767550e-02},{-2.533961e-02,6.522262e-02},{2.542743e-02,-6.517686e-02},{-2.530329e-02,6.285018e-02},{2.541361e-02,-6.283681e-02},{-2.527047e-02,6.062533e-02},{2.540105e-02,-6.063985e-02},{-2.524071e-02,5.853392e-02},{2.538962e-02,-5.857244e-02},{-2.521363e-02,5.656353e-02},{2.537919e-02,-5.662273e-02},{-2.518894e-02,5.470329e-02},{2.536963e-02,-5.478026e-02},{-2.516634e-02,5.294356e-02},{2.536087e-02,-5.303581e-02},{-2.514561e-02,5.127583e-02},{2.535280e-02,-5.138116e-02},{-2.512656e-02,4.969252e-02},{2.534536e-02,-4.980902e-02},{-2.510899e-02,4.818686e-02},{2.533849e-02,-4.831285e-02},{-2.509277e-02,4.675278e-02},{2.533212e-02,-4.688680e-02},{-2.507776e-02,4.538485e-02},{2.532622e-02,-4.552559e-02},{-2.506384e-02,4.407817e-02},{2.532074e-02,-4.422447e-02},{-2.505090e-02,4.282829e-02},{2.531563e-02,-4.297914e-02},{-2.503887e-02,4.163120e-02},{2.531087e-02,-4.178571e-02},{-2.502765e-02,4.048326e-02},{2.530643e-02,-4.064061e-02},{-2.501717e-02,3.938114e-02},{2.530227e-02,-3.954062e-02},{-2.500738e-02,3.832183e-02},{2.529838e-02,-3.848280e-02},{-2.499821e-02,3.730254e-02},{2.529473e-02,-3.746443e-02},{-2.498961e-02,3.632075e-02},{2.529130e-02,-3.648305e-02},{-2.498153e-02,3.537412e-02},{2.528808e-02,-3.553637e-02},{-2.497395e-02,3.446051e-02},{2.528505e-02,-3.462230e-02},{-2.496681e-02,3.357795e-02},{2.528220e-02,-3.373891e-02},{-2.496008e-02,3.272461e-02},{2.527950e-02,-3.288441e-02},{-2.495373e-02,3.189882e-02},{2.527696e-02,-3.205715e-02},{-2.494774e-02,3.109901e-02},{2.527456e-02,-3.125560e-02},{-2.494208e-02,3.032373e-02},{2.527228e-02,-3.047834e-02},{-2.493673e-02,2.957164e-02},{2.527013e-02,-2.972405e-02},{-2.493166e-02,2.884149e-02},{2.526809e-02,-2.899151e-02},{-2.492685e-02,2.813212e-02},{2.526616e-02,-2.827955e-02},{-2.492230e-02,2.744244e-02},{2.526432e-02,-2.758713e-02},{-2.491798e-02,2.677144e-02},{2.526258e-02,-2.691323e-02},{-2.491387e-02,2.611816e-02},{2.526092e-02,-2.625692e-02},{-2.490997e-02,2.548172e-02},{2.525935e-02,-2.561734e-02},{-2.490626e-02,2.486130e-02},{2.525785e-02,-2.499364e-02},{-2.490273e-02,2.425609e-02},{2.525642e-02,-2.438508e-02},{-2.489936e-02,2.366539e-02},{2.525506e-02,-2.379091e-02},{-2.489616e-02,2.308848e-02},{2.525376e-02,-2.321046e-02},{-2.489311e-02,2.252473e-02},{2.525253e-02,-2.264309e-02},{-2.489019e-02,2.197351e-02},{2.525135e-02,-2.208819e-02},{-2.488741e-02,2.143426e-02},{2.525022e-02,-2.154519e-02},{-2.488476e-02,2.090643e-02},{2.524914e-02,-2.101354e-02},{-2.488222e-02,2.038950e-02},{2.524812e-02,-2.049275e-02},{-2.487980e-02,1.988299e-02},{2.524713e-02,-1.998231e-02},{-2.487748e-02,1.938642e-02},{2.524619e-02,-1.948179e-02},{-2.487527e-02,1.889937e-02},{2.524529e-02,-1.899074e-02},{-2.487315e-02,1.842142e-02},{2.524443e-02,-1.850874e-02},{-2.487112e-02,1.795217e-02},{2.524361e-02,-1.803541e-02},{-2.486918e-02,1.749125e-02},{2.524282e-02,-1.757038e-02},{-2.486733e-02,1.703830e-02},{2.524207e-02,-1.711329e-02},{-2.486555e-02,1.659298e-02},{2.524134e-02,-1.666380e-02},{-2.486385e-02,1.615497e-02},{2.524065e-02,-1.622159e-02},{-2.486221e-02,1.572396e-02},{2.523999e-02,-1.578635e-02},{-2.486065e-02,1.529965e-02},{2.523935e-02,-1.535780e-02},{-2.485915e-02,1.488177e-02},{2.523874e-02,-1.493564e-02},{-2.485772e-02,1.447003e-02},{2.523816e-02,-1.451962e-02},{-2.485634e-02,1.406420e-02},{2.523760e-02,-1.410947e-02},{-2.485503e-02,1.366401e-02},{2.523706e-02,-1.370495e-02},{-2.485376e-02,1.326924e-02},{2.523655e-02,-1.330582e-02},{-2.485256e-02,1.287965e-02},{2.523606e-02,-1.291187e-02},{-2.485140e-02,1.249503e-02},{2.523558e-02,-1.252286e-02},{-2.485029e-02,1.211517e-02},{2.523513e-02,-1.213859e-02},{-2.484922e-02,1.173986e-02},{2.523470e-02,-1.175886e-02},{-2.484821e-02,1.136892e-02},{2.523428e-02,-1.138348e-02},{-2.484723e-02,1.100216e-02},{2.523389e-02,-1.101226e-02},{-2.484630e-02,1.063940e-02},{2.523351e-02,-1.064502e-02},{-2.484541e-02,1.028046e-02},{2.523315e-02,-1.028158e-02},{-2.484456e-02,9.925172e-03},{2.523280e-02,-9.921789e-03},{-2.484374e-02,9.573383e-03},{2.523247e-02,-9.565472e-03},{-2.484297e-02,9.224933e-03},{2.523215e-02,-9.212476e-03},{-2.484222e-02,8.879669e-03},{2.523185e-02,-8.862648e-03},{-2.484152e-02,8.537444e-03},{2.523156e-02,-8.515841e-03},{-2.484084e-02,8.198117e-03},{2.523129e-02,-8.171912e-03},{-2.484020e-02,7.861549e-03},{2.523103e-02,-7.830723e-03},{-2.483959e-02,7.527605e-03},{2.523078e-02,-7.492138e-03},{-2.483901e-02,7.196155e-03},{2.523055e-02,-7.156026e-03},{-2.483846e-02,6.867071e-03},{2.523032e-02,-6.822260e-03},{-2.483794e-02,6.540230e-03},{2.523011e-02,-6.490714e-03},{-2.483745e-02,6.215510e-03},{2.522991e-02,-6.161268e-03},{-2.483699e-02,5.892793e-03},{2.522973e-02,-5.833801e-03},{-2.483655e-02,5.571964e-03},{2.522955e-02,-5.508198e-03},{-2.483614e-02,5.252910e-03},{2.522938e-02,-5.184345e-03},{-2.483576e-02,4.935520e-03},{2.522923e-02,-4.862131e-03},{-2.483540e-02,4.619686e-03},{2.522909e-02,-4.541445e-03},{-2.483506e-02,4.305300e-03},{2.522895e-02,-4.222181e-03},{-2.483476e-02,3.992259e-03},{2.522883e-02,-3.904234e-03},{-2.483447e-02,3.680460e-03},{2.522872e-02,-3.587498e-03},{-2.483421e-02,3.369801e-03},{2.522861e-02,-3.271872e-03},{-2.483398e-02,3.060182e-03},{2.522852e-02,-2.957254e-03},{-2.483376e-02,2.751505e-03},{2.522844e-02,-2.643546e-03},{-2.483357e-02,2.443673e-03},{2.522836e-02,-2.330647e-03},{-2.483341e-02,2.136588e-03},{2.522830e-02,-2.018461e-03},{-2.483326e-02,1.830155e-03},{2.522824e-02,-1.706891e-03},{-2.483314e-02,1.524280e-03},{2.522819e-02,-1.395841e-03},{-2.483304e-02,1.218869e-03},{2.522816e-02,-1.085215e-03},{-2.483296e-02,9.138286e-04},{2.522813e-02,-7.749183e-04},{-2.483291e-02,6.090652e-04},{2.522811e-02,-4.648570e-04},{-2.483288e-02,3.044864e-04},{2.522810e-02,-1.549367e-04},{-2.483287e-02,0.000000e+00},{2.522810e-02,1.549367e-04},{-2.483288e-02,-3.044864e-04},{2.522811e-02,4.648570e-04},{-2.483291e-02,-6.090652e-04},{2.522813e-02,7.749183e-04},{-2.483296e-02,-9.138286e-04},{2.522816e-02,1.085215e-03},{-2.483304e-02,-1.218869e-03},{2.522819e-02,1.395841e-03},{-2.483314e-02,-1.524280e-03},{2.522824e-02,1.706891e-03},{-2.483326e-02,-1.830155e-03},{2.522830e-02,2.018461e-03},{-2.483341e-02,-2.136588e-03},{2.522836e-02,2.330647e-03},{-2.483357e-02,-2.443673e-03},{2.522844e-02,2.643546e-03},{-2.483376e-02,-2.751505e-03},{2.522852e-02,2.957254e-03},{-2.483398e-02,-3.060182e-03},{2.522861e-02,3.271872e-03},{-2.483421e-02,-3.369801e-03},{2.522872e-02,3.587498e-03},{-2.483447e-02,-3.680460e-03},{2.522883e-02,3.904234e-03},{-2.483476e-02,-3.992259e-03},{2.522895e-02,4.222181e-03},{-2.483506e-02,-4.305300e-03},{2.522909e-02,4.541445e-03},{-2.483540e-02,-4.619686e-03},{2.522923e-02,4.862131e-03},{-2.483576e-02,-4.935520e-03},{2.522938e-02,5.184345e-03},{-2.483614e-02,-5.252910e-03},{2.522955e-02,5.508198e-03},{-2.483655e-02,-5.571964e-03},{2.522973e-02,5.833801e-03},{-2.483699e-02,-5.892793e-03},{2.522991e-02,6.161268e-03},{-2.483745e-02,-6.215510e-03},{2.523011e-02,6.490714e-03},{-2.483794e-02,-6.540230e-03},{2.523032e-02,6.822260e-03},{-2.483846e-02,-6.867071e-03},{2.523055e-02,7.156026e-03},{-2.483901e-02,-7.196155e-03},{2.523078e-02,7.492138e-03},{-2.483959e-02,-7.527605e-03},{2.523103e-02,7.830723e-03},{-2.484020e-02,-7.861549e-03},{2.523129e-02,8.171912e-03},{-2.484084e-02,-8.198117e-03},{2.523156e-02,8.515841e-03},{-2.484152e-02,-8.537444e-03},{2.523185e-02,8.862648e-03},{-2.484222e-02,-8.879669e-03},{2.523215e-02,9.212476e-03},{-2.484297e-02,-9.224933e-03},{2.523247e-02,9.565472e-03},{-2.484374e-02,-9.573383e-03},{2.523280e-02,9.921789e-03},{-2.484456e-02,-9.925172e-03},{2.523315e-02,1.028158e-02},{-2.484541e-02,-1.028046e-02},{2.523351e-02,1.064502e-02},{-2.484630e-02,-1.063940e-02},{2.523389e-02,1.101226e-02},{-2.484723e-02,-1.100216e-02},{2.523428e-02,1.138348e-02},{-2.484821e-02,-1.136892e-02},{2.523470e-02,1.175886e-02},{-2.484922e-02,-1.173986e-02},{2.523513e-02,1.213859e-02},{-2.485029e-02,-1.211517e-02},{2.523558e-02,1.252286e-02},{-2.485140e-02,-1.249503e-02},{2.523606e-02,1.291187e-02},{-2.485256e-02,-1.287965e-02},{2.523655e-02,1.330582e-02},{-2.485376e-02,-1.326924e-02},{2.523706e-02,1.370495e-02},{-2.485503e-02,-1.366401e-02},{2.523760e-02,1.410947e-02},{-2.485634e-02,-1.406420e-02},{2.523816e-02,1.451962e-02},{-2.485772e-02,-1.447003e-02},{2.523874e-02,1.493564e-02},{-2.485915e-02,-1.488177e-02},{2.523935e-02,1.535780e-02},{-2.486065e-02,-1.529965e-02},{2.523999e-02,1.578635e-02},{-2.486221e-02,-1.572396e-02},{2.524065e-02,1.622159e-02},{-2.486385e-02,-1.615497e-02},{2.524134e-02,1.666380e-02},{-2.486555e-02,-1.659298e-02},{2.524207e-02,1.711329e-02},{-2.486733e-02,-1.703830e-02},{2.524282e-02,1.757038e-02},{-2.486918e-02,-1.749125e-02},{2.524361e-02,1.803541e-02},{-2.487112e-02,-1.795217e-02},{2.524443e-02,1.850874e-02},{-2.487315e-02,-1.842142e-02},{2.524529e-02,1.899074e-02},{-2.487527e-02,-1.889937e-02},{2.524619e-02,1.948179e-02},{-2.487748e-02,-1.938642e-02},{2.524713e-02,1.998231e-02},{-2.487980e-02,-1.988299e-02},{2.524812e-02,2.049275e-02},{-2.488222e-02,-2.038950e-02},{2.524914e-02,2.101354e-02},{-2.488476e-02,-2.090643e-02},{2.525022e-02,2.154519e-02},{-2.488741e-02,-2.143426e-02},{2.525135e-02,2.208819e-02},{-2.489019e-02,-2.197351e-02},{2.525253e-02,2.264309e-02},{-2.489311e-02,-2.252473e-02},{2.525376e-02,2.321046e-02},{-2.489616e-02,-2.308848e-02},{2.525506e-02,2.379091e-02},{-2.489936e-02,-2.366539e-02},{2.525642e-02,2.438508e-02},{-2.490273e-02,-2.425609e-02},{2.525785e-02,2.499364e-02},{-2.490626e-02,-2.486130e-02},{2.525935e-02,2.561734e-02},{-2.490997e-02,-2.548172e-02},{2.526092e-02,2.625692e-02},{-2.491387e-02,-2.611816e-02},{2.526258e-02,2.691323e-02},{-2.491798e-02,-2.677144e-02},{2.526432e-02,2.758713e-02},{-2.492230e-02,-2.744244e-02},{2.526616e-02,2.827955e-02},{-2.492685e-02,-2.813212e-02},{2.526809e-02,2.899151e-02},{-2.493166e-02,-2.884149e-02},{2.527013e-02,2.972405e-02},{-2.493673e-02,-2.957164e-02},{2.527228e-02,3.047834e-02},{-2.494208e-02,-3.032373e-02},{2.527456e-02,3.125560e-02},{-2.494774e-02,-3.109901e-02},{2.527696e-02,3.205715e-02},{-2.495373e-02,-3.189882e-02},{2.527950e-02,3.288441e-02},{-2.496008e-02,-3.272461e-02},{2.528220e-02,3.373891e-02},{-2.496681e-02,-3.357795e-02},{2.528505e-02,3.462230e-02},{-2.497395e-02,-3.446051e-02},{2.528808e-02,3.553637e-02},{-2.498153e-02,-3.537412e-02},{2.529130e-02,3.648305e-02},{-2.498961e-02,-3.632075e-02},{2.529473e-02,3.746443e-02},{-2.499821e-02,-3.730254e-02},{2.529838e-02,3.848280e-02},{-2.500738e-02,-3.832183e-02},{2.530227e-02,3.954062e-02},{-2.501717e-02,-3.938114e-02},{2.530643e-02,4.064061e-02},{-2.502765e-02,-4.048326e-02},{2.531087e-02,4.178571e-02},{-2.503887e-02,-4.163120e-02},{2.531563e-02,4.297914e-02},{-2.505090e-02,-4.282829e-02},{2.532074e-02,4.422447e-02},{-2.506384e-02,-4.407817e-02},{2.532622e-02,4.552559e-02},{-2.507776e-02,-4.538485e-02},{2.533212e-02,4.688680e-02},{-2.509277e-02,-4.675278e-02},{2.533849e-02,4.831285e-02},{-2.510899e-02,-4.818686e-02},{2.534536e-02,4.980902e-02},{-2.512656e-02,-4.969252e-02},{2.535280e-02,5.138116e-02},{-2.514561e-02,-5.127583e-02},{2.536087e-02,5.303581e-02},{-2.516634e-02,-5.294356e-02},{2.536963e-02,5.478026e-02},{-2.518894e-02,-5.470329e-02},{2.537919e-02,5.662273e-02},{-2.521363e-02,-5.656353e-02},{2.538962e-02,5.857244e-02},{-2.524071e-02,-5.853392e-02},{2.540105e-02,6.063985e-02},{-2.527047e-02,-6.062533e-02},{2.541361e-02,6.283681e-02},{-2.530329e-02,-6.285018e-02},{2.542743e-02,6.517686e-02},{-2.533961e-02,-6.522262e-02},{2.544272e-02,6.767550e-02},{-2.537995e-02,-6.775894e-02},{2.545967e-02,7.035059e-02},{-2.542494e-02,-7.047793e-02},{2.547853e-02,7.322283e-02},{-2.547531e-02,-7.340143e-02},{2.549962e-02,7.631634e-02},{-2.553200e-02,-7.655495e-02},{2.552329e-02,7.965942e-02},{-2.559610e-02,-7.996854e-02},{2.554998e-02,8.328550e-02},{-2.566900e-02,-8.367780e-02},{2.558023e-02,8.723443e-02},{-2.575243e-02,-8.772535e-02},{2.561471e-02,9.155408e-02},{-2.584854e-02,-9.216259e-02},{2.565424e-02,9.630258e-02},{-2.596012e-02,-9.705222e-02},{2.569987e-02,1.015512e-01},{-2.609077e-02,-1.024716e-01},{2.575291e-02,1.073887e-01},{-2.624525e-02,-1.085175e-01},{2.581508e-02,1.139267e-01},{-2.642993e-02,-1.153126e-01},{2.588858e-02,1.213084e-01},{-2.665356e-02,-1.230155e-01},{2.597635e-02,1.297208e-01},{-2.692848e-02,-1.318355e-01},{2.608229e-02,1.394138e-01},{-2.727255e-02,-1.420551e-01},{2.621171e-02,1.507303e-01},{-2.771266e-02,-1.540680e-01},{2.637185e-02,1.641575e-01},{-2.829116e-02,-1.684429e-01},{2.657263e-02,1.804162e-01},{-2.907877e-02,-1.860415e-01},{2.682713e-02,2.006342e-01},{-3.020323e-02,-2.082545e-01},{2.715000e-02,2.267108e-01},{-3.192231e-02,-2.375351e-01},{2.754295e-02,2.622079e-01},{-3.485256e-02,-2.788376e-01},{2.789522e-02,3.150421e-01},{-4.097981e-02,-3.448773e-01},{2.704656e-02,4.094653e-01},{-6.292887e-02,-4.970809e-01},{-1.079679e-01,7.509862e-01},{4.555769e-01,-6.293127e-01},{-4.634791e-01,4.347373e-01},{4.851274e-01,-3.723914e-01},{-4.783659e-01,3.013647e-01},{4.928898e-01,-2.616729e-01},{-4.868308e-01,2.146209e-01},{4.978219e-01,-1.824441e-01},{-4.920420e-01,1.453929e-01},{5.010047e-01,-1.164641e-01},{-4.951476e-01,8.457449e-02},{5.028096e-01,-5.688214e-02},{-4.966120e-01,2.778743e-02}};
//complex_float B3[512] = {{5.033960e-01,0.000000e+00},{4.969156e-01,-2.169115e-02},{5.040542e-01,-4.452544e-02},{4.979251e-01,-6.629217e-02},{5.061159e-01,-9.174069e-02},{5.000316e-01,-1.149469e-01},{5.098944e-01,-1.453277e-01},{5.034496e-01,-1.720608e-01},{5.161648e-01,-2.121012e-01},{5.086683e-01,-2.468019e-01},{5.270637e-01,-3.102061e-01},{5.177700e-01,-3.684070e-01},{5.429853e-01,-5.556543e-01},{2.258972e-01,-7.242982e-01},{2.297948e-02,-5.005211e-01},{4.835886e-02,-4.074981e-01},{2.708983e-02,-3.462454e-01},{3.795842e-02,-3.139886e-01},{2.708794e-02,-2.796986e-01},{3.378762e-02,-2.614766e-01},{2.675070e-02,-2.381730e-01},{3.152740e-02,-2.261436e-01},{2.643499e-02,-2.087663e-01},{3.011497e-02,-2.001671e-01},{2.617833e-02,-1.864717e-01},{2.915491e-02,-1.800170e-01},{2.597420e-02,-1.688157e-01},{2.846469e-02,-1.638075e-01},{2.581126e-02,-1.543979e-01},{2.794801e-02,-1.504181e-01},{2.567984e-02,-1.423517e-01},{2.754917e-02,-1.391313e-01},{2.557259e-02,-1.321052e-01},{2.723375e-02,-1.294627e-01},{2.548403e-02,-1.232631e-01},{2.697934e-02,-1.210705e-01},{2.541011e-02,-1.155416e-01},{2.677076e-02,-1.137059e-01},{2.534779e-02,-1.087306e-01},{2.659736e-02,-1.071827e-01},{2.529477e-02,-1.026710e-01},{2.645148e-02,-1.013581e-01},{2.524930e-02,-9.723957e-02},{2.632748e-02,-9.612071e-02},{2.521001e-02,-9.233929e-02},{2.622111e-02,-9.138225e-02},{2.517582e-02,-8.789256e-02},{2.612913e-02,-8.707159e-02},{2.514590e-02,-8.383649e-02},{2.604902e-02,-8.313077e-02},{2.511955e-02,-8.011951e-02},{2.597878e-02,-7.951204e-02},{2.509624e-02,-7.669891e-02},{2.591685e-02,-7.617566e-02},{2.507550e-02,-7.353897e-02},{2.586194e-02,-7.308829e-02},{2.505699e-02,-7.060959e-02},{2.581303e-02,-7.022170e-02},{2.504038e-02,-6.788516e-02},{2.576925e-02,-6.755183e-02},{2.502542e-02,-6.534382e-02},{2.572992e-02,-6.505804e-02},{2.501191e-02,-6.296670e-02},{2.569443e-02,-6.272250e-02},{2.499967e-02,-6.073750e-02},{2.566231e-02,-6.052975e-02},{2.498853e-02,-5.864202e-02},{2.563314e-02,-5.846628e-02},{2.497837e-02,-5.666782e-02},{2.560657e-02,-5.652026e-02},{2.496908e-02,-5.480399e-02},{2.558229e-02,-5.468127e-02},{2.496057e-02,-5.304088e-02},{2.556005e-02,-5.294010e-02},{2.495274e-02,-5.136997e-02},{2.553962e-02,-5.128855e-02},{2.494553e-02,-4.978364e-02},{2.552082e-02,-4.971935e-02},{2.493888e-02,-4.827512e-02},{2.550347e-02,-4.822596e-02},{2.493272e-02,-4.683833e-02},{2.548743e-02,-4.680256e-02},{2.492702e-02,-4.546782e-02},{2.547257e-02,-4.544387e-02},{2.492172e-02,-4.415868e-02},{2.545878e-02,-4.414515e-02},{2.491679e-02,-4.290645e-02},{2.544596e-02,-4.290211e-02},{2.491220e-02,-4.170712e-02},{2.543401e-02,-4.171087e-02},{2.490792e-02,-4.055703e-02},{2.542287e-02,-4.056787e-02},{2.490391e-02,-3.945286e-02},{2.541246e-02,-3.946990e-02},{2.490017e-02,-3.839157e-02},{2.540271e-02,-3.841401e-02},{2.489666e-02,-3.737039e-02},{2.539359e-02,-3.739750e-02},{2.489336e-02,-3.638678e-02},{2.538502e-02,-3.641790e-02},{2.489027e-02,-3.543840e-02},{2.537698e-02,-3.547294e-02},{2.488735e-02,-3.452310e-02},{2.536941e-02,-3.456053e-02},{2.488461e-02,-3.363891e-02},{2.536229e-02,-3.367874e-02},{2.488203e-02,-3.278400e-02},{2.535557e-02,-3.282579e-02},{2.487959e-02,-3.195668e-02},{2.534923e-02,-3.200003e-02},{2.487728e-02,-3.115540e-02},{2.534325e-02,-3.119993e-02},{2.487511e-02,-3.037869e-02},{2.533759e-02,-3.042407e-02},{2.487305e-02,-2.962522e-02},{2.533223e-02,-2.967114e-02},{2.487109e-02,-2.889374e-02},{2.532716e-02,-2.893992e-02},{2.486924e-02,-2.818307e-02},{2.532235e-02,-2.822925e-02},{2.486749e-02,-2.749212e-02},{2.531779e-02,-2.753807e-02},{2.486582e-02,-2.681989e-02},{2.531346e-02,-2.686538e-02},{2.486423e-02,-2.616542e-02},{2.530935e-02,-2.621025e-02},{2.486273e-02,-2.552782e-02},{2.530544e-02,-2.557181e-02},{2.486130e-02,-2.490626e-02},{2.530171e-02,-2.494924e-02},{2.485993e-02,-2.429995e-02},{2.529817e-02,-2.434176e-02},{2.485863e-02,-2.370817e-02},{2.529480e-02,-2.374866e-02},{2.485740e-02,-2.313021e-02},{2.529158e-02,-2.316925e-02},{2.485621e-02,-2.256543e-02},{2.528852e-02,-2.260289e-02},{2.485509e-02,-2.201321e-02},{2.528559e-02,-2.204898e-02},{2.485401e-02,-2.147298e-02},{2.528280e-02,-2.150695e-02},{2.485299e-02,-2.094419e-02},{2.528013e-02,-2.097626e-02},{2.485201e-02,-2.042632e-02},{2.527758e-02,-2.045639e-02},{2.485107e-02,-1.991889e-02},{2.527515e-02,-1.994687e-02},{2.485017e-02,-1.942142e-02},{2.527282e-02,-1.944724e-02},{2.484931e-02,-1.893349e-02},{2.527059e-02,-1.895706e-02},{2.484849e-02,-1.845467e-02},{2.526846e-02,-1.847592e-02},{2.484771e-02,-1.798457e-02},{2.526642e-02,-1.800344e-02},{2.484696e-02,-1.752281e-02},{2.526447e-02,-1.753923e-02},{2.484624e-02,-1.706904e-02},{2.526261e-02,-1.708295e-02},{2.484555e-02,-1.662291e-02},{2.526082e-02,-1.663426e-02},{2.484489e-02,-1.618411e-02},{2.525910e-02,-1.619284e-02},{2.484426e-02,-1.575232e-02},{2.525746e-02,-1.575838e-02},{2.484365e-02,-1.532724e-02},{2.525589e-02,-1.533059e-02},{2.484307e-02,-1.490860e-02},{2.525438e-02,-1.490918e-02},{2.484251e-02,-1.449612e-02},{2.525294e-02,-1.449390e-02},{2.484198e-02,-1.408955e-02},{2.525155e-02,-1.408448e-02},{2.484147e-02,-1.368864e-02},{2.525023e-02,-1.368068e-02},{2.484098e-02,-1.329316e-02},{2.524896e-02,-1.328226e-02},{2.484051e-02,-1.290286e-02},{2.524774e-02,-1.288900e-02},{2.484006e-02,-1.251755e-02},{2.524658e-02,-1.250068e-02},{2.483963e-02,-1.213700e-02},{2.524546e-02,-1.211709e-02},{2.483922e-02,-1.176102e-02},{2.524439e-02,-1.173804e-02},{2.483882e-02,-1.138941e-02},{2.524337e-02,-1.136333e-02},{2.483845e-02,-1.102199e-02},{2.524239e-02,-1.099276e-02},{2.483808e-02,-1.065857e-02},{2.524145e-02,-1.062617e-02},{2.483774e-02,-1.029898e-02},{2.524055e-02,-1.026338e-02},{2.483741e-02,-9.943053e-03},{2.523970e-02,-9.904227e-03},{2.483709e-02,-9.590629e-03},{2.523888e-02,-9.548542e-03},{2.483679e-02,-9.241550e-03},{2.523810e-02,-9.196171e-03},{2.483650e-02,-8.895663e-03},{2.523735e-02,-8.846962e-03},{2.483623e-02,-8.552822e-03},{2.523664e-02,-8.500770e-03},{2.483596e-02,-8.212883e-03},{2.523597e-02,-8.157450e-03},{2.483571e-02,-7.875708e-03},{2.523533e-02,-7.816865e-03},{2.483548e-02,-7.541162e-03},{2.523471e-02,-7.478880e-03},{2.483525e-02,-7.209115e-03},{2.523413e-02,-7.143364e-03},{2.483504e-02,-6.879438e-03},{2.523358e-02,-6.810189e-03},{2.483484e-02,-6.552007e-03},{2.523306e-02,-6.479230e-03},{2.483465e-02,-6.226702e-03},{2.523257e-02,-6.150367e-03},{2.483447e-02,-5.903404e-03},{2.523211e-02,-5.823480e-03},{2.483430e-02,-5.581997e-03},{2.523168e-02,-5.498453e-03},{2.483414e-02,-5.262368e-03},{2.523127e-02,-5.175173e-03},{2.483399e-02,-4.944406e-03},{2.523089e-02,-4.853529e-03},{2.483385e-02,-4.628003e-03},{2.523053e-02,-4.533411e-03},{2.483372e-02,-4.313052e-03},{2.523020e-02,-4.214712e-03},{2.483360e-02,-3.999447e-03},{2.522990e-02,-3.897327e-03},{2.483349e-02,-3.687086e-03},{2.522962e-02,-3.581152e-03},{2.483339e-02,-3.375868e-03},{2.522936e-02,-3.266084e-03},{2.483330e-02,-3.065692e-03},{2.522913e-02,-2.952023e-03},{2.483321e-02,-2.756459e-03},{2.522892e-02,-2.638869e-03},{2.483314e-02,-2.448072e-03},{2.522874e-02,-2.326524e-03},{2.483308e-02,-2.140434e-03},{2.522858e-02,-2.014891e-03},{2.483302e-02,-1.833450e-03},{2.522844e-02,-1.703872e-03},{2.483297e-02,-1.527024e-03},{2.522833e-02,-1.393372e-03},{2.483293e-02,-1.221064e-03},{2.522824e-02,-1.083295e-03},{2.483290e-02,-9.154737e-04},{2.522817e-02,-7.735476e-04},{2.483288e-02,-6.101616e-04},{2.522813e-02,-4.640347e-04},{2.483287e-02,-3.050346e-04},{2.522810e-02,-1.546626e-04},{2.483287e-02,0.000000e+00},{2.522810e-02,1.546626e-04},{2.483287e-02,3.050346e-04},{2.522813e-02,4.640347e-04},{2.483288e-02,6.101616e-04},{2.522817e-02,7.735476e-04},{2.483290e-02,9.154737e-04},{2.522824e-02,1.083295e-03},{2.483293e-02,1.221064e-03},{2.522833e-02,1.393372e-03},{2.483297e-02,1.527024e-03},{2.522844e-02,1.703872e-03},{2.483302e-02,1.833450e-03},{2.522858e-02,2.014891e-03},{2.483308e-02,2.140434e-03},{2.522874e-02,2.326524e-03},{2.483314e-02,2.448072e-03},{2.522892e-02,2.638869e-03},{2.483321e-02,2.756459e-03},{2.522913e-02,2.952023e-03},{2.483330e-02,3.065692e-03},{2.522936e-02,3.266084e-03},{2.483339e-02,3.375868e-03},{2.522962e-02,3.581152e-03},{2.483349e-02,3.687086e-03},{2.522990e-02,3.897327e-03},{2.483360e-02,3.999447e-03},{2.523020e-02,4.214712e-03},{2.483372e-02,4.313052e-03},{2.523053e-02,4.533411e-03},{2.483385e-02,4.628003e-03},{2.523089e-02,4.853529e-03},{2.483399e-02,4.944406e-03},{2.523127e-02,5.175173e-03},{2.483414e-02,5.262368e-03},{2.523168e-02,5.498453e-03},{2.483430e-02,5.581997e-03},{2.523211e-02,5.823480e-03},{2.483447e-02,5.903404e-03},{2.523257e-02,6.150367e-03},{2.483465e-02,6.226702e-03},{2.523306e-02,6.479230e-03},{2.483484e-02,6.552007e-03},{2.523358e-02,6.810189e-03},{2.483504e-02,6.879438e-03},{2.523413e-02,7.143364e-03},{2.483525e-02,7.209115e-03},{2.523471e-02,7.478880e-03},{2.483548e-02,7.541162e-03},{2.523533e-02,7.816865e-03},{2.483571e-02,7.875708e-03},{2.523597e-02,8.157450e-03},{2.483596e-02,8.212883e-03},{2.523664e-02,8.500770e-03},{2.483623e-02,8.552822e-03},{2.523735e-02,8.846962e-03},{2.483650e-02,8.895663e-03},{2.523810e-02,9.196171e-03},{2.483679e-02,9.241550e-03},{2.523888e-02,9.548542e-03},{2.483709e-02,9.590629e-03},{2.523970e-02,9.904227e-03},{2.483741e-02,9.943053e-03},{2.524055e-02,1.026338e-02},{2.483774e-02,1.029898e-02},{2.524145e-02,1.062617e-02},{2.483808e-02,1.065857e-02},{2.524239e-02,1.099276e-02},{2.483845e-02,1.102199e-02},{2.524337e-02,1.136333e-02},{2.483882e-02,1.138941e-02},{2.524439e-02,1.173804e-02},{2.483922e-02,1.176102e-02},{2.524546e-02,1.211709e-02},{2.483963e-02,1.213700e-02},{2.524658e-02,1.250068e-02},{2.484006e-02,1.251755e-02},{2.524774e-02,1.288900e-02},{2.484051e-02,1.290286e-02},{2.524896e-02,1.328226e-02},{2.484098e-02,1.329316e-02},{2.525023e-02,1.368068e-02},{2.484147e-02,1.368864e-02},{2.525155e-02,1.408448e-02},{2.484198e-02,1.408955e-02},{2.525294e-02,1.449390e-02},{2.484251e-02,1.449612e-02},{2.525438e-02,1.490918e-02},{2.484307e-02,1.490860e-02},{2.525589e-02,1.533059e-02},{2.484365e-02,1.532724e-02},{2.525746e-02,1.575838e-02},{2.484426e-02,1.575232e-02},{2.525910e-02,1.619284e-02},{2.484489e-02,1.618411e-02},{2.526082e-02,1.663426e-02},{2.484555e-02,1.662291e-02},{2.526261e-02,1.708295e-02},{2.484624e-02,1.706904e-02},{2.526447e-02,1.753923e-02},{2.484696e-02,1.752281e-02},{2.526642e-02,1.800344e-02},{2.484771e-02,1.798457e-02},{2.526846e-02,1.847592e-02},{2.484849e-02,1.845467e-02},{2.527059e-02,1.895706e-02},{2.484931e-02,1.893349e-02},{2.527282e-02,1.944724e-02},{2.485017e-02,1.942142e-02},{2.527515e-02,1.994687e-02},{2.485107e-02,1.991889e-02},{2.527758e-02,2.045639e-02},{2.485201e-02,2.042632e-02},{2.528013e-02,2.097626e-02},{2.485299e-02,2.094419e-02},{2.528280e-02,2.150695e-02},{2.485401e-02,2.147298e-02},{2.528559e-02,2.204898e-02},{2.485509e-02,2.201321e-02},{2.528852e-02,2.260289e-02},{2.485621e-02,2.256543e-02},{2.529158e-02,2.316925e-02},{2.485740e-02,2.313021e-02},{2.529480e-02,2.374866e-02},{2.485863e-02,2.370817e-02},{2.529817e-02,2.434176e-02},{2.485993e-02,2.429995e-02},{2.530171e-02,2.494924e-02},{2.486130e-02,2.490626e-02},{2.530544e-02,2.557181e-02},{2.486273e-02,2.552782e-02},{2.530935e-02,2.621025e-02},{2.486423e-02,2.616542e-02},{2.531346e-02,2.686538e-02},{2.486582e-02,2.681989e-02},{2.531779e-02,2.753807e-02},{2.486749e-02,2.749212e-02},{2.532235e-02,2.822925e-02},{2.486924e-02,2.818307e-02},{2.532716e-02,2.893992e-02},{2.487109e-02,2.889374e-02},{2.533223e-02,2.967114e-02},{2.487305e-02,2.962522e-02},{2.533759e-02,3.042407e-02},{2.487511e-02,3.037869e-02},{2.534325e-02,3.119993e-02},{2.487728e-02,3.115540e-02},{2.534923e-02,3.200003e-02},{2.487959e-02,3.195668e-02},{2.535557e-02,3.282579e-02},{2.488203e-02,3.278400e-02},{2.536229e-02,3.367874e-02},{2.488461e-02,3.363891e-02},{2.536941e-02,3.456053e-02},{2.488735e-02,3.452310e-02},{2.537698e-02,3.547294e-02},{2.489027e-02,3.543840e-02},{2.538502e-02,3.641790e-02},{2.489336e-02,3.638678e-02},{2.539359e-02,3.739750e-02},{2.489666e-02,3.737039e-02},{2.540271e-02,3.841401e-02},{2.490017e-02,3.839157e-02},{2.541246e-02,3.946990e-02},{2.490391e-02,3.945286e-02},{2.542287e-02,4.056787e-02},{2.490792e-02,4.055703e-02},{2.543401e-02,4.171087e-02},{2.491220e-02,4.170712e-02},{2.544596e-02,4.290211e-02},{2.491679e-02,4.290645e-02},{2.545878e-02,4.414515e-02},{2.492172e-02,4.415868e-02},{2.547257e-02,4.544387e-02},{2.492702e-02,4.546782e-02},{2.548743e-02,4.680256e-02},{2.493272e-02,4.683833e-02},{2.550347e-02,4.822596e-02},{2.493888e-02,4.827512e-02},{2.552082e-02,4.971935e-02},{2.494553e-02,4.978364e-02},{2.553962e-02,5.128855e-02},{2.495274e-02,5.136997e-02},{2.556005e-02,5.294010e-02},{2.496057e-02,5.304088e-02},{2.558229e-02,5.468127e-02},{2.496908e-02,5.480399e-02},{2.560657e-02,5.652026e-02},{2.497837e-02,5.666782e-02},{2.563314e-02,5.846628e-02},{2.498853e-02,5.864202e-02},{2.566231e-02,6.052975e-02},{2.499967e-02,6.073750e-02},{2.569443e-02,6.272250e-02},{2.501191e-02,6.296670e-02},{2.572992e-02,6.505804e-02},{2.502542e-02,6.534382e-02},{2.576925e-02,6.755183e-02},{2.504038e-02,6.788516e-02},{2.581303e-02,7.022170e-02},{2.505699e-02,7.060959e-02},{2.586194e-02,7.308829e-02},{2.507550e-02,7.353897e-02},{2.591685e-02,7.617566e-02},{2.509624e-02,7.669891e-02},{2.597878e-02,7.951204e-02},{2.511955e-02,8.011951e-02},{2.604902e-02,8.313077e-02},{2.514590e-02,8.383649e-02},{2.612913e-02,8.707159e-02},{2.517582e-02,8.789256e-02},{2.622111e-02,9.138225e-02},{2.521001e-02,9.233929e-02},{2.632748e-02,9.612071e-02},{2.524930e-02,9.723957e-02},{2.645148e-02,1.013581e-01},{2.529477e-02,1.026710e-01},{2.659736e-02,1.071827e-01},{2.534779e-02,1.087306e-01},{2.677076e-02,1.137059e-01},{2.541011e-02,1.155416e-01},{2.697934e-02,1.210705e-01},{2.548403e-02,1.232631e-01},{2.723375e-02,1.294627e-01},{2.557259e-02,1.321052e-01},{2.754917e-02,1.391313e-01},{2.567984e-02,1.423517e-01},{2.794801e-02,1.504181e-01},{2.581126e-02,1.543979e-01},{2.846469e-02,1.638075e-01},{2.597420e-02,1.688157e-01},{2.915491e-02,1.800170e-01},{2.617833e-02,1.864717e-01},{3.011497e-02,2.001671e-01},{2.643499e-02,2.087663e-01},{3.152740e-02,2.261436e-01},{2.675070e-02,2.381730e-01},{3.378762e-02,2.614766e-01},{2.708794e-02,2.796986e-01},{3.795842e-02,3.139886e-01},{2.708983e-02,3.462454e-01},{4.835886e-02,4.074981e-01},{2.297948e-02,5.005211e-01},{2.258972e-01,7.242982e-01},{5.429853e-01,5.556543e-01},{5.177700e-01,3.684070e-01},{5.270637e-01,3.102061e-01},{5.086683e-01,2.468019e-01},{5.161648e-01,2.121012e-01},{5.034496e-01,1.720608e-01},{5.098944e-01,1.453277e-01},{5.000316e-01,1.149469e-01},{5.061159e-01,9.174069e-02},{4.979251e-01,6.629217e-02},{5.040542e-01,4.452544e-02},{4.969156e-01,2.169115e-02}};
//complex_float B4[512] = {{-3.395951e-03,0.000000e+00},{-3.227509e-03,1.840075e-04},{-3.477885e-03,4.680256e-04},{-3.386068e-03,5.854464e-04},{-3.747051e-03,1.021879e-03},{-3.750917e-03,1.109367e-03},{-4.291595e-03,1.807070e-03},{-4.457467e-03,1.948637e-03},{-5.346980e-03,3.198160e-03},{-5.875985e-03,3.741592e-03},{-7.575362e-03,6.708927e-03},{-8.833133e-03,1.065964e-02},{-4.524422e-03,2.424044e-02},{1.928899e-02,2.167041e-02},{1.959691e-02,3.598707e-04},{1.079507e-02,-2.455248e-03},{7.538174e-03,-3.693411e-03},{5.205702e-03,-2.364203e-03},{4.357856e-03,-2.926988e-03},{3.306530e-03,-1.931340e-03},{2.989167e-03,-2.374116e-03},{2.374199e-03,-1.617618e-03},{2.243127e-03,-1.999020e-03},{1.828673e-03,-1.392700e-03},{1.779945e-03,-1.731071e-03},{1.474721e-03,-1.225012e-03},{1.467712e-03,-1.530110e-03},{1.228845e-03,-1.095205e-03},{1.244939e-03,-1.373431e-03},{1.049544e-03,-9.915580e-04},{1.079239e-03,-1.247499e-03},{9.139460e-04,-9.066972e-04},{9.519975e-04,-1.143799e-03},{8.084490e-04,-8.357859e-04},{8.517904e-04,-1.056715e-03},{7.244818e-04,-7.755273e-04},{7.712372e-04,-9.823990e-04},{6.563892e-04,-7.235981e-04},{7.053700e-04,-9.181198e-04},{6.002982e-04,-6.783133e-04},{6.507302e-04,-8.618857e-04},{5.534735e-04,-6.384206e-04},{6.048412e-04,-8.122079e-04},{5.139325e-04,-6.029686e-04},{5.658867e-04,-7.679501e-04},{4.802058e-04,-5.712212e-04},{5.325071e-04,-7.282290e-04},{4.511836e-04,-5.425991e-04},{5.036662e-04,-6.923467e-04},{4.260130e-04,-5.166403e-04},{4.785617e-04,-6.597444e-04},{4.040294e-04,-4.929708e-04},{4.565641e-04,-6.299686e-04},{3.847077e-04,-4.712850e-04},{4.371728e-04,-6.026473e-04},{3.676283e-04,-4.513303e-04},{4.199858e-04,-5.774723e-04},{3.524522e-04,-4.328958e-04},{4.046767e-04,-5.541857e-04},{3.389031e-04,-4.158044e-04},{3.909781e-04,-5.325702e-04},{3.267537e-04,-3.999057e-04},{3.786691e-04,-5.124411e-04},{3.158154e-04,-3.850713e-04},{3.675659e-04,-4.936403e-04},{3.059308e-04,-3.711910e-04},{3.575146e-04,-4.760319e-04},{2.969676e-04,-3.581695e-04},{3.483852e-04,-4.594981e-04},{2.888137e-04,-3.459238e-04},{3.400674e-04,-4.439363e-04},{2.813739e-04,-3.343816e-04},{3.324671e-04,-4.292568e-04},{2.745666e-04,-3.234794e-04},{3.255037e-04,-4.153807e-04},{2.683217e-04,-3.131610e-04},{3.191075e-04,-4.022383e-04},{2.625786e-04,-3.033769e-04},{3.132183e-04,-3.897680e-04},{2.572849e-04,-2.940830e-04},{3.077839e-04,-3.779148e-04},{2.523946e-04,-2.852399e-04},{3.027583e-04,-3.666297e-04},{2.478677e-04,-2.768125e-04},{2.981017e-04,-3.558687e-04},{2.436692e-04,-2.687692e-04},{2.937787e-04,-3.455923e-04},{2.397679e-04,-2.610814e-04},{2.897583e-04,-3.357650e-04},{2.361366e-04,-2.537235e-04},{2.860129e-04,-3.263544e-04},{2.327510e-04,-2.466722e-04},{2.825181e-04,-3.173316e-04},{2.295895e-04,-2.399064e-04},{2.792522e-04,-3.086699e-04},{2.266329e-04,-2.334068e-04},{2.761957e-04,-3.003452e-04},{2.238639e-04,-2.271559e-04},{2.733313e-04,-2.923354e-04},{2.212673e-04,-2.211376e-04},{2.706433e-04,-2.846205e-04},{2.188290e-04,-2.153372e-04},{2.681178e-04,-2.771819e-04},{2.165367e-04,-2.097413e-04},{2.657420e-04,-2.700026e-04},{2.143791e-04,-2.043374e-04},{2.635045e-04,-2.630669e-04},{2.123459e-04,-1.991141e-04},{2.613950e-04,-2.563606e-04},{2.104281e-04,-1.940609e-04},{2.594042e-04,-2.498702e-04},{2.086172e-04,-1.891679e-04},{2.575233e-04,-2.435834e-04},{2.069056e-04,-1.844261e-04},{2.557448e-04,-2.374889e-04},{2.052864e-04,-1.798271e-04},{2.540615e-04,-2.315760e-04},{2.037531e-04,-1.753633e-04},{2.524669e-04,-2.258348e-04},{2.023001e-04,-1.710272e-04},{2.509551e-04,-2.202563e-04},{2.009220e-04,-1.668121e-04},{2.495207e-04,-2.148319e-04},{1.996140e-04,-1.627118e-04},{2.481587e-04,-2.095535e-04},{1.983715e-04,-1.587203e-04},{2.468646e-04,-2.044138e-04},{1.971905e-04,-1.548322e-04},{2.456340e-04,-1.994057e-04},{1.960671e-04,-1.510423e-04},{2.444631e-04,-1.945227e-04},{1.949980e-04,-1.473457e-04},{2.433484e-04,-1.897587e-04},{1.939797e-04,-1.437380e-04},{2.422865e-04,-1.851080e-04},{1.930095e-04,-1.402148e-04},{2.412743e-04,-1.805650e-04},{1.920844e-04,-1.367721e-04},{2.403090e-04,-1.761248e-04},{1.912020e-04,-1.334062e-04},{2.393880e-04,-1.717825e-04},{1.903598e-04,-1.301134e-04},{2.385088e-04,-1.675336e-04},{1.895558e-04,-1.268906e-04},{2.376692e-04,-1.633738e-04},{1.887877e-04,-1.237343e-04},{2.368670e-04,-1.592990e-04},{1.880537e-04,-1.206417e-04},{2.361002e-04,-1.553055e-04},{1.873521e-04,-1.176099e-04},{2.353672e-04,-1.513897e-04},{1.866811e-04,-1.146362e-04},{2.346660e-04,-1.475480e-04},{1.860393e-04,-1.117181e-04},{2.339952e-04,-1.437772e-04},{1.854251e-04,-1.088530e-04},{2.333532e-04,-1.400743e-04},{1.848373e-04,-1.060388e-04},{2.327387e-04,-1.364363e-04},{1.842745e-04,-1.032732e-04},{2.321503e-04,-1.328605e-04},{1.837356e-04,-1.005542e-04},{2.315868e-04,-1.293440e-04},{1.832195e-04,-9.787969e-05},{2.310471e-04,-1.258845e-04},{1.827252e-04,-9.524784e-05},{2.305302e-04,-1.224795e-04},{1.822516e-04,-9.265684e-05},{2.300349e-04,-1.191267e-04},{1.817979e-04,-9.010496e-05},{2.295603e-04,-1.158238e-04},{1.813632e-04,-8.759055e-05},{2.291057e-04,-1.125688e-04},{1.809466e-04,-8.511203e-05},{2.286701e-04,-1.093597e-04},{1.805476e-04,-8.266789e-05},{2.282527e-04,-1.061945e-04},{1.801652e-04,-8.025667e-05},{2.278528e-04,-1.030714e-04},{1.797989e-04,-7.787699e-05},{2.274698e-04,-9.998850e-05},{1.794481e-04,-7.552751e-05},{2.271029e-04,-9.694422e-05},{1.791120e-04,-7.320696e-05},{2.267515e-04,-9.393688e-05},{1.787903e-04,-7.091409e-05},{2.264152e-04,-9.096489e-05},{1.784823e-04,-6.864773e-05},{2.260932e-04,-8.802675e-05},{1.781876e-04,-6.640673e-05},{2.257852e-04,-8.512098e-05},{1.779056e-04,-6.418999e-05},{2.254906e-04,-8.224617e-05},{1.776361e-04,-6.199645e-05},{2.252090e-04,-7.940096e-05},{1.773784e-04,-5.982509e-05},{2.249399e-04,-7.658404e-05},{1.771323e-04,-5.767490e-05},{2.246830e-04,-7.379413e-05},{1.768974e-04,-5.554493e-05},{2.244378e-04,-7.103000e-05},{1.766733e-04,-5.343426e-05},{2.242040e-04,-6.829045e-05},{1.764597e-04,-5.134199e-05},{2.239813e-04,-6.557433e-05},{1.762563e-04,-4.926723e-05},{2.237693e-04,-6.288052e-05},{1.760629e-04,-4.720915e-05},{2.235678e-04,-6.020793e-05},{1.758790e-04,-4.516692e-05},{2.233764e-04,-5.755549e-05},{1.757046e-04,-4.313973e-05},{2.231949e-04,-5.492218e-05},{1.755393e-04,-4.112682e-05},{2.230231e-04,-5.230699e-05},{1.753829e-04,-3.912741e-05},{2.228607e-04,-4.970894e-05},{1.752352e-04,-3.714078e-05},{2.227075e-04,-4.712707e-05},{1.750961e-04,-3.516618e-05},{2.225633e-04,-4.456045e-05},{1.749652e-04,-3.320292e-05},{2.224279e-04,-4.200816e-05},{1.748426e-04,-3.125030e-05},{2.223011e-04,-3.946932e-05},{1.747279e-04,-2.930764e-05},{2.221828e-04,-3.694303e-05},{1.746211e-04,-2.737428e-05},{2.220728e-04,-3.442845e-05},{1.745220e-04,-2.544956e-05},{2.219709e-04,-3.192472e-05},{1.744304e-04,-2.353285e-05},{2.218772e-04,-2.943102e-05},{1.743464e-04,-2.162350e-05},{2.217913e-04,-2.694652e-05},{1.742697e-04,-1.972089e-05},{2.217133e-04,-2.447042e-05},{1.742003e-04,-1.782441e-05},{2.216429e-04,-2.200192e-05},{1.741380e-04,-1.593346e-05},{2.215802e-04,-1.954023e-05},{1.740829e-04,-1.404743e-05},{2.215251e-04,-1.708458e-05},{1.740348e-04,-1.216573e-05},{2.214775e-04,-1.463420e-05},{1.739937e-04,-1.028778e-05},{2.214372e-04,-1.218832e-05},{1.739595e-04,-8.412978e-06},{2.214044e-04,-9.746174e-06},{1.739322e-04,-6.540759e-06},{2.213789e-04,-7.307021e-06},{1.739118e-04,-4.670542e-06},{2.213607e-04,-4.870105e-06},{1.738982e-04,-2.801755e-06},{2.213498e-04,-2.434680e-06},{1.738914e-04,-9.338231e-07},{2.213462e-04,0.000000e+00},{1.738914e-04,9.338231e-07},{2.213498e-04,2.434680e-06},{1.738982e-04,2.801755e-06},{2.213607e-04,4.870105e-06},{1.739118e-04,4.670542e-06},{2.213789e-04,7.307021e-06},{1.739322e-04,6.540759e-06},{2.214044e-04,9.746174e-06},{1.739595e-04,8.412978e-06},{2.214372e-04,1.218832e-05},{1.739937e-04,1.028778e-05},{2.214775e-04,1.463420e-05},{1.740348e-04,1.216573e-05},{2.215251e-04,1.708458e-05},{1.740829e-04,1.404743e-05},{2.215802e-04,1.954023e-05},{1.741380e-04,1.593346e-05},{2.216429e-04,2.200192e-05},{1.742003e-04,1.782441e-05},{2.217133e-04,2.447042e-05},{1.742697e-04,1.972089e-05},{2.217913e-04,2.694652e-05},{1.743464e-04,2.162350e-05},{2.218772e-04,2.943102e-05},{1.744304e-04,2.353285e-05},{2.219709e-04,3.192472e-05},{1.745220e-04,2.544956e-05},{2.220728e-04,3.442845e-05},{1.746211e-04,2.737428e-05},{2.221828e-04,3.694303e-05},{1.747279e-04,2.930764e-05},{2.223011e-04,3.946932e-05},{1.748426e-04,3.125030e-05},{2.224279e-04,4.200816e-05},{1.749652e-04,3.320292e-05},{2.225633e-04,4.456045e-05},{1.750961e-04,3.516618e-05},{2.227075e-04,4.712707e-05},{1.752352e-04,3.714078e-05},{2.228607e-04,4.970894e-05},{1.753829e-04,3.912741e-05},{2.230231e-04,5.230699e-05},{1.755393e-04,4.112682e-05},{2.231949e-04,5.492218e-05},{1.757046e-04,4.313973e-05},{2.233764e-04,5.755549e-05},{1.758790e-04,4.516692e-05},{2.235678e-04,6.020793e-05},{1.760629e-04,4.720915e-05},{2.237693e-04,6.288052e-05},{1.762563e-04,4.926723e-05},{2.239813e-04,6.557433e-05},{1.764597e-04,5.134199e-05},{2.242040e-04,6.829045e-05},{1.766733e-04,5.343426e-05},{2.244378e-04,7.103000e-05},{1.768974e-04,5.554493e-05},{2.246830e-04,7.379413e-05},{1.771323e-04,5.767490e-05},{2.249399e-04,7.658404e-05},{1.773784e-04,5.982509e-05},{2.252090e-04,7.940096e-05},{1.776361e-04,6.199645e-05},{2.254906e-04,8.224617e-05},{1.779056e-04,6.418999e-05},{2.257852e-04,8.512098e-05},{1.781876e-04,6.640673e-05},{2.260932e-04,8.802675e-05},{1.784823e-04,6.864773e-05},{2.264152e-04,9.096489e-05},{1.787903e-04,7.091409e-05},{2.267515e-04,9.393688e-05},{1.791120e-04,7.320696e-05},{2.271029e-04,9.694422e-05},{1.794481e-04,7.552751e-05},{2.274698e-04,9.998850e-05},{1.797989e-04,7.787699e-05},{2.278528e-04,1.030714e-04},{1.801652e-04,8.025667e-05},{2.282527e-04,1.061945e-04},{1.805476e-04,8.266789e-05},{2.286701e-04,1.093597e-04},{1.809466e-04,8.511203e-05},{2.291057e-04,1.125688e-04},{1.813632e-04,8.759055e-05},{2.295603e-04,1.158238e-04},{1.817979e-04,9.010496e-05},{2.300349e-04,1.191267e-04},{1.822516e-04,9.265684e-05},{2.305302e-04,1.224795e-04},{1.827252e-04,9.524784e-05},{2.310471e-04,1.258845e-04},{1.832195e-04,9.787969e-05},{2.315868e-04,1.293440e-04},{1.837356e-04,1.005542e-04},{2.321503e-04,1.328605e-04},{1.842745e-04,1.032732e-04},{2.327387e-04,1.364363e-04},{1.848373e-04,1.060388e-04},{2.333532e-04,1.400743e-04},{1.854251e-04,1.088530e-04},{2.339952e-04,1.437772e-04},{1.860393e-04,1.117181e-04},{2.346660e-04,1.475480e-04},{1.866811e-04,1.146362e-04},{2.353672e-04,1.513897e-04},{1.873521e-04,1.176099e-04},{2.361002e-04,1.553055e-04},{1.880537e-04,1.206417e-04},{2.368670e-04,1.592990e-04},{1.887877e-04,1.237343e-04},{2.376692e-04,1.633738e-04},{1.895558e-04,1.268906e-04},{2.385088e-04,1.675336e-04},{1.903598e-04,1.301134e-04},{2.393880e-04,1.717825e-04},{1.912020e-04,1.334062e-04},{2.403090e-04,1.761248e-04},{1.920844e-04,1.367721e-04},{2.412743e-04,1.805650e-04},{1.930095e-04,1.402148e-04},{2.422865e-04,1.851080e-04},{1.939797e-04,1.437380e-04},{2.433484e-04,1.897587e-04},{1.949980e-04,1.473457e-04},{2.444631e-04,1.945227e-04},{1.960671e-04,1.510423e-04},{2.456340e-04,1.994057e-04},{1.971905e-04,1.548322e-04},{2.468646e-04,2.044138e-04},{1.983715e-04,1.587203e-04},{2.481587e-04,2.095535e-04},{1.996140e-04,1.627118e-04},{2.495207e-04,2.148319e-04},{2.009220e-04,1.668121e-04},{2.509551e-04,2.202563e-04},{2.023001e-04,1.710272e-04},{2.524669e-04,2.258348e-04},{2.037531e-04,1.753633e-04},{2.540615e-04,2.315760e-04},{2.052864e-04,1.798271e-04},{2.557448e-04,2.374889e-04},{2.069056e-04,1.844261e-04},{2.575233e-04,2.435834e-04},{2.086172e-04,1.891679e-04},{2.594042e-04,2.498702e-04},{2.104281e-04,1.940609e-04},{2.613950e-04,2.563606e-04},{2.123459e-04,1.991141e-04},{2.635045e-04,2.630669e-04},{2.143791e-04,2.043374e-04},{2.657420e-04,2.700026e-04},{2.165367e-04,2.097413e-04},{2.681178e-04,2.771819e-04},{2.188290e-04,2.153372e-04},{2.706433e-04,2.846205e-04},{2.212673e-04,2.211376e-04},{2.733313e-04,2.923354e-04},{2.238639e-04,2.271559e-04},{2.761957e-04,3.003452e-04},{2.266329e-04,2.334068e-04},{2.792522e-04,3.086699e-04},{2.295895e-04,2.399064e-04},{2.825181e-04,3.173316e-04},{2.327510e-04,2.466722e-04},{2.860129e-04,3.263544e-04},{2.361366e-04,2.537235e-04},{2.897583e-04,3.357650e-04},{2.397679e-04,2.610814e-04},{2.937787e-04,3.455923e-04},{2.436692e-04,2.687692e-04},{2.981017e-04,3.558687e-04},{2.478677e-04,2.768125e-04},{3.027583e-04,3.666297e-04},{2.523946e-04,2.852399e-04},{3.077839e-04,3.779148e-04},{2.572849e-04,2.940830e-04},{3.132183e-04,3.897680e-04},{2.625786e-04,3.033769e-04},{3.191075e-04,4.022383e-04},{2.683217e-04,3.131610e-04},{3.255037e-04,4.153807e-04},{2.745666e-04,3.234794e-04},{3.324671e-04,4.292568e-04},{2.813739e-04,3.343816e-04},{3.400674e-04,4.439363e-04},{2.888137e-04,3.459238e-04},{3.483852e-04,4.594981e-04},{2.969676e-04,3.581695e-04},{3.575146e-04,4.760319e-04},{3.059308e-04,3.711910e-04},{3.675659e-04,4.936403e-04},{3.158154e-04,3.850713e-04},{3.786691e-04,5.124411e-04},{3.267537e-04,3.999057e-04},{3.909781e-04,5.325702e-04},{3.389031e-04,4.158044e-04},{4.046767e-04,5.541857e-04},{3.524522e-04,4.328958e-04},{4.199858e-04,5.774723e-04},{3.676283e-04,4.513303e-04},{4.371728e-04,6.026473e-04},{3.847077e-04,4.712850e-04},{4.565641e-04,6.299686e-04},{4.040294e-04,4.929708e-04},{4.785617e-04,6.597444e-04},{4.260130e-04,5.166403e-04},{5.036662e-04,6.923467e-04},{4.511836e-04,5.425991e-04},{5.325071e-04,7.282290e-04},{4.802058e-04,5.712212e-04},{5.658867e-04,7.679501e-04},{5.139325e-04,6.029686e-04},{6.048412e-04,8.122079e-04},{5.534735e-04,6.384206e-04},{6.507302e-04,8.618857e-04},{6.002982e-04,6.783133e-04},{7.053700e-04,9.181198e-04},{6.563892e-04,7.235981e-04},{7.712372e-04,9.823990e-04},{7.244818e-04,7.755273e-04},{8.517904e-04,1.056715e-03},{8.084490e-04,8.357859e-04},{9.519975e-04,1.143799e-03},{9.139460e-04,9.066972e-04},{1.079239e-03,1.247499e-03},{1.049544e-03,9.915580e-04},{1.244939e-03,1.373431e-03},{1.228845e-03,1.095205e-03},{1.467712e-03,1.530110e-03},{1.474721e-03,1.225012e-03},{1.779945e-03,1.731071e-03},{1.828673e-03,1.392700e-03},{2.243127e-03,1.999020e-03},{2.374199e-03,1.617618e-03},{2.989167e-03,2.374116e-03},{3.306530e-03,1.931340e-03},{4.357856e-03,2.926988e-03},{5.205702e-03,2.364203e-03},{7.538174e-03,3.693411e-03},{1.079507e-02,2.455248e-03},{1.959691e-02,-3.598707e-04},{1.928899e-02,-2.167041e-02},{-4.524422e-03,-2.424044e-02},{-8.833133e-03,-1.065964e-02},{-7.575362e-03,-6.708927e-03},{-5.875985e-03,-3.741592e-03},{-5.346980e-03,-3.198160e-03},{-4.457467e-03,-1.948637e-03},{-4.291595e-03,-1.807070e-03},{-3.750917e-03,-1.109367e-03},{-3.747051e-03,-1.021879e-03},{-3.386068e-03,-5.854464e-04},{-3.477885e-03,-4.680256e-04},{-3.227509e-03,-1.840075e-04}};
//
//float blockInputBuffer[1280] = {0};
//float blockOutputBuffer[256] = {0};
//float middleBuffer[256] = {0};
//int blockCnt = 0;



//float phase = 0;
//float output = 0;
//float sinFreq = 100.0f;
//int capDelay = 0;
//#define CAP_DELAY (96000)
//float freqTable[240]={100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500,1600,1700,1800,1900,2000,2100,2200,2300,2400,2500,2600,2700,2800,2900,3000,3100,3200,3300,3400,3500,3600,3700,3800,3900,4000,4100,4200,4300,4400,4500,4600,4700,4800,4900,5000,5100,5200,5300,5400,5500,5600,5700,5800,5900,6000,6100,6200,6300,6400,6500,6600,6700,6800,6900,7000,7100,7200,7300,7400,7500,7600,7700,7800,7900,8000,8100,8200,8300,8400,8500,8600,8700,8800,8900,9000,9100,9200,9300,9400,9500,9600,9700,9800,9900,10000,10100,10200,10300,10400,10500,10600,10700,10800,10900,11000,11100,11200,11300,11400,11500,11600,11700,11800,11900,12000,12100,12200,12300,12400,12500,12600,12700,12800,12900,13000,13100,13200,13300,13400,13500,13600,13700,13800,13900,14000,14100,14200,14300,14400,14500,14600,14700,14800,14900,15000,15100,15200,15300,15400,15500,15600,15700,15800,15900,16000,16100,16200,16300,16400,16500,16600,16700,16800,16900,17000,17100,17200,17300,17400,17500,17600,17700,17800,17900,18000,18100,18200,18300,18400,18500,18600,18700,18800,18900,19000,19100,19200,19300,19400,19500,19600,19700,19800,19900,20000,20100,20200,20300,20400,20500,20600,20700,20800,20900,21000,21100,21200,21300,21400,21500,21600,21700,21800,21900,22000,22100,22200,22300,22400,22500,22600,22700,22800,22900,23000,23100,23200,23300,23400,23500,23600,23700,23800,23900,24000};
//
//int freqCnt=0;
//
//float delayBuffer[240] = {0};
//int delayCnt = 0;
//
//
//int capCnt = 0;
//
//
//float xCap[SAMPLE_LENGTH] = {0};
//float yCap[SAMPLE_LENGTH] = {0};
//float xAmp=0;
//float yAmp=0;
//float correlation[CORR_LAG];
//float mag[240];
//float phs[240];

//float firBuffer[480]={0};
//float firFilter[480]={-1.66440383033168e-05,-4.85294027089635e-05,-7.60171572173651e-05,-9.64296350380978e-05,-0.000107716910929695,-0.000108653812208205,-9.89664436064594e-05,-7.93757981070271e-05,-5.15527437346831e-05,-1.79860642859716e-05,1.82272647746900e-05,5.36533592980099e-05,8.48321087411620e-05,0.000108600762234143,0.000122401658248179,0.000124545459926861,0.000114403905316066,9.25111855317865e-05,6.05603355781688e-05,2.12899572333202e-05,-2.17335047557680e-05,-6.44226839897553e-05,-0.000102540929020275,-0.000132106655900171,-0.000149793448759979,-0.000153287175802541,-0.000141563225430863,-0.000115052485538151,-7.56736126998368e-05,-2.67208343569560e-05,2.73899601090687e-05,8.14999638559169e-05,0.000130180791439046,0.000168260944946535,0.000191356124980731,0.000196350716566076,0.000181779460401736,0.000148065236955241,9.75806906964134e-05,3.45172263160586e-05,-3.54366301846132e-05,-0.000105586106766482,-0.000168850470014151,-0.000218457389719141,-0.000248645249895480,-0.000255302568674989,-0.000236475945513056,-0.000192687364403054,-0.000127017712141984,-4.49347683823370e-05,4.61311959746592e-05,0.000137435073419920,0.000219734181700678,0.000284201091541223,0.000323344038218762,0.000331841258535921,0.000307199237858414,0.000250157962670473,0.000164788026164381,5.82530581356715e-05,-5.97561350365033e-05,-0.000177875297375222,-0.000284136191684327,-0.000367153717436342,-0.000417315345270889,-0.000427851262378928,-0.000395669542268129,-0.000321858787680747,-0.000211789806035949,-7.47855210037279e-05,7.66289486026573e-05,0.000227840452095933,0.000363530487774563,0.000469198383077273,0.000532676220601488,0.000545480286459942,0.000503852977030823,0.000409373705750227,0.000269055615722778,9.48935436694475e-05,-9.71168135496674e-05,-0.000288413534691994,-0.000459631968400690,-0.000592532660190697,-0.000671904813243525,-0.000687250126301719,-0.000634065288232089,-0.000514574059531913,-0.000337809259373092,-0.000119006794526428,0.000121657663740643,0.000360890343973168,0.000574499036517948,0.000739802738207417,0.000837994730756169,0.000856216900030619,0.000789121919529340,0.000639742330442559,0.000419548233730811,0.000147652732965624,-0.000150790844690227,-0.000446871911806160,-0.000710683202133478,-0.000914299349981206,-0.00103468080845852,-0.00105620578399513,-0.000972558221875430,-0.000787753908489373,-0.000516165137368945,-0.000181500139997144,0.000185202418342217,0.000548401370968451,0.000871451044585371,0.00112024905351771,0.00126677545951666,0.00129216146937042,0.00118895894466125,0.000962349678952986,0.000630130158482801,0.000221424702566847,-0.000225793585111211,-0.000668171171671617,-0.00106112110953839,-0.00136325748380658,-0.00154068185839157,-0.00157068430906030,-0.00144446369550633,-0.00116855536201379,-0.000764772614050094,-0.000268610492115432,0.000273786869447385,0.000809845645000806,0.00128558996945111,0.00165100367914734,0.00186520040940821,0.00190087565073338,0.00174756656791625,0.00141334717294305,0.000924729426397243,0.000324712189935213,-0.000330896485926544,-0.000978580443987389,-0.00155318256054638,-0.00199436671974304,-0.00225284247632909,-0.00229572037290994,-0.00211042924721811,-0.00170674946676891,-0.00111668776739813,-0.000392124877887831,0.000399612912984470,0.00118189408561250,0.00187608543075967,0.00240933352345567,0.00272206554905296,0.00277445050572537,0.00255113701624569,0.00206373006511043,0.00135067398746544,0.000474454740566024,-0.000483702043251299,-0.00143120500759867,-0.00227288851410031,-0.00292040145507356,-0.00330128289498180,-0.00336681048105614,-0.00309779449244137,-0.00250766250318920,-0.00164242304438114,-0.000577389953046117,0.000589134815905723,0.00174471359707199,0.00277338401988895,0.00356704796898235,0.00403654266851058,0.00412128635639860,0.00379648783261313,0.00307710909208914,0.00201805758324515,0.000710434539629964,-0.000725953538283382,-0.00215323784752107,-0.00342836910664023,-0.00441706571765647,-0.00500750301643386,-0.00512238872666211,-0.00472817084699909,-0.00384035135352399,-0.00252421522079601,-0.000890701129166475,0.000912399232270930,0.00271326480201500,0.00433182505853340,0.00559709659924284,0.00636446440939264,0.00653122921239499,0.00604883362614534,0.00493042682717296,0.00325281287767976,0.00115232087898257,-0.00118530656503941,-0.00354033645032620,-0.00567857427407197,-0.00737334709355569,-0.00842797832599674,-0.00869665064877529,-0.00810160168013789,-0.00664483818140280,-0.00441297385668941,-0.00157436205141933,0.00163163821138715,0.00491271320451176,0.00794772039410583,0.0104150368683279,0.0120228078228136,0.0125384902142875,0.0118150241266161,0.00981113525270348,0.00660366940465557,0.00239045702552863,-0.00251704327450314,-0.00771122857138384,-0.0127150016252039,-0.0170158094218575,-0.0201045995279368,-0.0215167167130284,-0.0208714903234840,-0.0179072551989734,-0.0125088128903045,-0.00472485982929980,0.00522635299574178,0.0169639655976409,0.0299639088892374,0.0435896120846361,0.0571319490410605,0.0698555849221632,0.0810483318417372,0.0900698349409589,0.0963959150933598,0.0996551933812599,0.0996551933812599,0.0963959150933598,0.0900698349409589,0.0810483318417372,0.0698555849221632,0.0571319490410605,0.0435896120846361,0.0299639088892374,0.0169639655976409,0.00522635299574178,-0.00472485982929980,-0.0125088128903045,-0.0179072551989734,-0.0208714903234840,-0.0215167167130284,-0.0201045995279368,-0.0170158094218575,-0.0127150016252039,-0.00771122857138384,-0.00251704327450314,0.00239045702552863,0.00660366940465557,0.00981113525270348,0.0118150241266161,0.0125384902142875,0.0120228078228136,0.0104150368683279,0.00794772039410583,0.00491271320451176,0.00163163821138715,-0.00157436205141933,-0.00441297385668941,-0.00664483818140280,-0.00810160168013789,-0.00869665064877529,-0.00842797832599674,-0.00737334709355569,-0.00567857427407197,-0.00354033645032620,-0.00118530656503941,0.00115232087898257,0.00325281287767976,0.00493042682717296,0.00604883362614534,0.00653122921239499,0.00636446440939264,0.00559709659924284,0.00433182505853340,0.00271326480201500,0.000912399232270930,-0.000890701129166475,-0.00252421522079601,-0.00384035135352399,-0.00472817084699909,-0.00512238872666211,-0.00500750301643386,-0.00441706571765647,-0.00342836910664023,-0.00215323784752107,-0.000725953538283382,0.000710434539629964,0.00201805758324515,0.00307710909208914,0.00379648783261313,0.00412128635639860,0.00403654266851058,0.00356704796898235,0.00277338401988895,0.00174471359707199,0.000589134815905723,-0.000577389953046117,-0.00164242304438114,-0.00250766250318920,-0.00309779449244137,-0.00336681048105614,-0.00330128289498180,-0.00292040145507356,-0.00227288851410031,-0.00143120500759867,-0.000483702043251299,0.000474454740566024,0.00135067398746544,0.00206373006511043,0.00255113701624569,0.00277445050572537,0.00272206554905296,0.00240933352345567,0.00187608543075967,0.00118189408561250,0.000399612912984470,-0.000392124877887831,-0.00111668776739813,-0.00170674946676891,-0.00211042924721811,-0.00229572037290994,-0.00225284247632909,-0.00199436671974304,-0.00155318256054638,-0.000978580443987389,-0.000330896485926544,0.000324712189935213,0.000924729426397243,0.00141334717294305,0.00174756656791625,0.00190087565073338,0.00186520040940821,0.00165100367914734,0.00128558996945111,0.000809845645000806,0.000273786869447385,-0.000268610492115432,-0.000764772614050094,-0.00116855536201379,-0.00144446369550633,-0.00157068430906030,-0.00154068185839157,-0.00136325748380658,-0.00106112110953839,-0.000668171171671617,-0.000225793585111211,0.000221424702566847,0.000630130158482801,0.000962349678952986,0.00118895894466125,0.00129216146937042,0.00126677545951666,0.00112024905351771,0.000871451044585371,0.000548401370968451,0.000185202418342217,-0.000181500139997144,-0.000516165137368945,-0.000787753908489373,-0.000972558221875430,-0.00105620578399513,-0.00103468080845852,-0.000914299349981206,-0.000710683202133478,-0.000446871911806160,-0.000150790844690227,0.000147652732965624,0.000419548233730811,0.000639742330442559,0.000789121919529340,0.000856216900030619,0.000837994730756169,0.000739802738207417,0.000574499036517948,0.000360890343973168,0.000121657663740643,-0.000119006794526428,-0.000337809259373092,-0.000514574059531913,-0.000634065288232089,-0.000687250126301719,-0.000671904813243525,-0.000592532660190697,-0.000459631968400690,-0.000288413534691994,-9.71168135496674e-05,9.48935436694475e-05,0.000269055615722778,0.000409373705750227,0.000503852977030823,0.000545480286459942,0.000532676220601488,0.000469198383077273,0.000363530487774563,0.000227840452095933,7.66289486026573e-05,-7.47855210037279e-05,-0.000211789806035949,-0.000321858787680747,-0.000395669542268129,-0.000427851262378928,-0.000417315345270889,-0.000367153717436342,-0.000284136191684327,-0.000177875297375222,-5.97561350365033e-05,5.82530581356715e-05,0.000164788026164381,0.000250157962670473,0.000307199237858414,0.000331841258535921,0.000323344038218762,0.000284201091541223,0.000219734181700678,0.000137435073419920,4.61311959746592e-05,-4.49347683823370e-05,-0.000127017712141984,-0.000192687364403054,-0.000236475945513056,-0.000255302568674989,-0.000248645249895480,-0.000218457389719141,-0.000168850470014151,-0.000105586106766482,-3.54366301846132e-05,3.45172263160586e-05,9.75806906964134e-05,0.000148065236955241,0.000181779460401736,0.000196350716566076,0.000191356124980731,0.000168260944946535,0.000130180791439046,8.14999638559169e-05,2.73899601090687e-05,-2.67208343569560e-05,-7.56736126998368e-05,-0.000115052485538151,-0.000141563225430863,-0.000153287175802541,-0.000149793448759979,-0.000132106655900171,-0.000102540929020275,-6.44226839897553e-05,-2.17335047557680e-05,2.12899572333202e-05,6.05603355781688e-05,9.25111855317865e-05,0.000114403905316066,0.000124545459926861,0.000122401658248179,0.000108600762234143,8.48321087411620e-05,5.36533592980099e-05,1.82272647746900e-05,-1.79860642859716e-05,-5.15527437346831e-05,-7.93757981070271e-05,-9.89664436064594e-05,-0.000108653812208205,-0.000107716910929695,-9.64296350380978e-05,-7.60171572173651e-05,-4.85294027089635e-05,-1.66440383033168e-05};
//int firBufferCounter=0;
////float firBuffer[32]={0};
////float firFilter[32] = {-0.00164669503127192,-0.00196744782733322,-0.00250036744003242,-0.00296842787715706,-0.00284469171718017,-0.00142778011510680,0.00202255863574004,0.00811479753363871,0.0171552631187992,0.0290163488422141,0.0430760835724918,0.0582487064351857,0.0731077802876000,0.0860840708042931,0.0957041110830604,0.100825689695059,0.100825689695059,0.0957041110830604,0.0860840708042931,0.0731077802876000,0.0582487064351857,0.0430760835724918,0.0290163488422141,0.0171552631187992,0.00811479753363871,0.00202255863574004,-0.00142778011510680,-0.00284469171718017,-0.00296842787715706,-0.00250036744003242,-0.00196744782733322,-0.00164669503127192};
//
//
//int capDelay = 0;
//#define CAP_DELAY (96000)
//float xCap[SYS_LENGTH] = {0};
//float yCap[SYS_LENGTH] = {0};
//int capCnt = 0;
//float firFit[SYS_LENGTH] = {0};
//float error;
//int epoch = 0;
//float output;
////float convergence[8000] = {0};
//int dst = 0;
//float temp[2] = {30000,30000};


/* anti-aliasing filter */
//float aaFilter[300] = {4.593977e-07,-9.454628e-06,9.574093e-06,5.198642e-06,-1.278898e-06,-6.819917e-06,-6.744686e-06,3.668798e-07,9.004946e-06,1.031485e-05,8.806741e-07,-1.200659e-05,-1.540756e-05,-3.291519e-06,1.529777e-05,2.225877e-05,7.547501e-06,-1.839872e-05,-3.105040e-05,-1.438679e-05,2.063823e-05,4.181248e-05,2.460096e-05,-2.109809e-05,-5.433395e-05,-3.898162e-05,1.858750e-05,6.808335e-05,5.823345e-05,-1.165815e-05,-8.211931e-05,-8.286294e-05,-1.353038e-06,9.502007e-05,1.130399e-04,2.224351e-05,-1.048332e-04,-1.484474e-04,-5.281834e-05,1.090594e-04,1.881273e-04,9.472047e-05,-1.046841e-04,-2.303281e-04,-1.492193e-04,8.826378e-05,2.723817e-04,2.169676e-04,-5.607744e-05,-3.106164e-04,-2.977356e-04,4.352437e-06,3.403331e-04,3.901421e-04,7.044818e-05,-3.558580e-04,-4.913974e-04,-1.712673e-04,3.506856e-04,5.970939e-04,3.000463e-04,-3.177281e-04,-7.010518e-04,-4.572873e-04,2.496621e-04,7.952636e-04,6.416099e-04,-1.393857e-04,-8.699583e-04,-8.493356e-04,-1.942261e-05,9.137821e-04,1.074147e-03,2.316638e-04,-9.141794e-04,-1.306729e-03,-5.001960e-04,8.578248e-04,1.534759e-03,8.250815e-04,-7.313223e-04,-1.742811e-03,-1.202963e-03,5.219373e-04,1.912596e-03,1.626466e-03,-2.184892e-04,-2.023321e-03,-2.083711e-03,-1.877196e-04,2.052257e-03,2.557984e-03,7.019350e-04,-1.975470e-03,-3.027565e-03,-1.325032e-03,1.768699e-03,3.465768e-03,2.052648e-03,-1.408311e-03,-3.841134e-03,-2.874464e-03,8.722992e-04,4.117780e-03,3.773624e-03,-1.412031e-04,-4.255832e-03,-4.726337e-03,-8.011249e-04,4.211810e-03,5.701625e-03,1.967043e-03,-3.938792e-03,-6.661165e-03,-3.365103e-03,3.386064e-03,7.559073e-03,5.000710e-03,-2.497775e-03,-8.341373e-03,-6.877821e-03,1.209768e-03,8.944635e-03,9.002290e-03,5.569221e-04,-9.292825e-03,-1.138803e-02,-2.909001e-03,9.290341e-03,1.406851e-02,6.006976e-03,-8.806729e-03,-1.711963e-02,-1.012079e-02,7.641636e-03,2.070963e-02,1.576052e-02,-5.436877e-03,-2.522540e-02,-2.403147e-02,1.418896e-03,3.166255e-02,3.784098e-02,6.575577e-03,-4.328675e-02,-6.792964e-02,-2.808988e-02,7.924307e-02,2.112636e-01,3.017091e-01,3.017091e-01,2.112636e-01,7.924307e-02,-2.808988e-02,-6.792964e-02,-4.328675e-02,6.575577e-03,3.784098e-02,3.166255e-02,1.418896e-03,-2.403147e-02,-2.522540e-02,-5.436877e-03,1.576052e-02,2.070963e-02,7.641636e-03,-1.012079e-02,-1.711963e-02,-8.806729e-03,6.006976e-03,1.406851e-02,9.290341e-03,-2.909001e-03,-1.138803e-02,-9.292825e-03,5.569221e-04,9.002290e-03,8.944635e-03,1.209768e-03,-6.877821e-03,-8.341373e-03,-2.497775e-03,5.000710e-03,7.559073e-03,3.386064e-03,-3.365103e-03,-6.661165e-03,-3.938792e-03,1.967043e-03,5.701625e-03,4.211810e-03,-8.011249e-04,-4.726337e-03,-4.255832e-03,-1.412031e-04,3.773624e-03,4.117780e-03,8.722992e-04,-2.874464e-03,-3.841134e-03,-1.408311e-03,2.052648e-03,3.465768e-03,1.768699e-03,-1.325032e-03,-3.027565e-03,-1.975470e-03,7.019350e-04,2.557984e-03,2.052257e-03,-1.877196e-04,-2.083711e-03,-2.023321e-03,-2.184892e-04,1.626466e-03,1.912596e-03,5.219373e-04,-1.202963e-03,-1.742811e-03,-7.313223e-04,8.250815e-04,1.534759e-03,8.578248e-04,-5.001960e-04,-1.306729e-03,-9.141794e-04,2.316638e-04,1.074147e-03,9.137821e-04,-1.942261e-05,-8.493356e-04,-8.699583e-04,-1.393857e-04,6.416099e-04,7.952636e-04,2.496621e-04,-4.572873e-04,-7.010518e-04,-3.177281e-04,3.000463e-04,5.970939e-04,3.506856e-04,-1.712673e-04,-4.913974e-04,-3.558580e-04,7.044818e-05,3.901421e-04,3.403331e-04,4.352437e-06,-2.977356e-04,-3.106164e-04,-5.607744e-05,2.169676e-04,2.723817e-04,8.826378e-05,-1.492193e-04,-2.303281e-04,-1.046841e-04,9.472047e-05,1.881273e-04,1.090594e-04,-5.281834e-05,-1.484474e-04,-1.048332e-04,2.224351e-05,1.130399e-04,9.502007e-05,-1.353038e-06,-8.286294e-05,-8.211931e-05,-1.165815e-05,5.823345e-05,6.808335e-05,1.858750e-05,-3.898162e-05,-5.433395e-05,-2.109809e-05,2.460096e-05,4.181248e-05,2.063823e-05,-1.438679e-05,-3.105040e-05,-1.839872e-05,7.547501e-06,2.225877e-05,1.529777e-05,-3.291519e-06,-1.540756e-05,-1.200659e-05,8.806741e-07,1.031485e-05,9.004946e-06,3.668798e-07,-6.744686e-06,-6.819917e-06,-1.278898e-06,5.198642e-06,9.574093e-06,-9.454628e-06,4.593977e-07};
//float aaFilter[300] = {-8.45011304875402e-05,-0.00207925902605286,-0.000227768927611145,-0.000203831166464738,-4.23373398720898e-05,0.000139638083216722,0.000301322189111397,0.000399146152793842,0.000403778285772824,0.000306018107911122,0.000123551936096403,-0.000104469482676730,-0.000322747046191756,-0.000476023611671503,-0.000519910225057264,-0.000435972930478074,-0.000235647756792349,3.80197801358169e-05,0.000321661726173334,0.000543595791653537,0.000644104835925005,0.000588844945912514,0.000381973784588710,6.49334891517613e-05,-0.000288951840645289,-0.000593492321711194,-0.000768131381123457,-0.000761179565529975,-0.000562283100847654,-0.000210083087778465,0.000217061369202846,0.000615835391598136,0.000884953858656829,0.000948313052515260,0.000778164443560065,0.000402617249195520,-9.57721550088415e-05,-0.000599851214765325,-0.000982925059372773,-0.00114242756498996,-0.00102480015587528,-0.000644283517924812,-8.07994050962511e-05,0.000534226190937332,0.00105026034065052,0.00133223670076952,0.00129735552689972,0.000936422436097352,0.000322183468886171,-0.000406447531171399,-0.00107300150830799,-0.00150778620717139,-0.00158898492169210,-0.00127789359379036,-0.000631676635553281,0.000205480435592108,0.00103578588223087,0.00165203444124591,0.00188853225913880,0.00166463397417807,0.00101556569271055,8.07346505198323e-05,-0.000924986301950368,-0.00174897886080622,-0.00218106334318668,-0.00209104798819840,-0.00147560705208410,-0.000461466156592793,0.000719168292560208,0.00178015334256871,0.00245160754544550,0.00254612673517478,0.00201148722794215,0.000948917484967274,-0.000403203461952063,-0.00172315033598271,-0.00268014785762408,-0.00301808363129534,-0.00262130441133404,-0.00155212468821863,-4.24245814635350e-05,0.00155380296687732,0.00284398455796397,0.00349112151810480,0.00330196592411329,0.00228075576371429,0.000639685555779194,-0.00124494075125038,-0.00291615727095717,-0.00394736724813684,-0.00404839890491124,-0.00314619916775268,-0.00141269996253721,0.000763817543823744,0.00286531730560341,0.00436434760724723,0.00485590227040089,0.00416296073858980,0.00239414379788477,-6.92633871522469e-05,-0.00265169080000238,-0.00471677657681364,-0.00572072307742373,-0.00535361761996745,-0.00362800246910812,-0.000895334632684916,0.00222293200660433,0.00497204473913865,0.00664450212322939,0.00675694317361097,0.00518482450401190,0.00221406259836600,-0.00150344842508819,-0.00508916383265880,-0.00763832593122044,-0.00844434474260682,-0.00718474600611743,-0.00402945969193484,0.000370694735887264,0.00500716542551709,0.00873707931656989,0.0105569693922753,0.00985833280468664,0.00660729642869476,0.00140348734599666,-0.00462116475466275,-0.0100287408975305,-0.0134041680526106,-0.0137001391812507,-0.0105285523000909,-0.00431738279400548,0.00370978630578411,0.0117453379754635,0.0177664824921516,0.0199936468732958,0.0173392600079425,0.00974957079401820,-0.00164793135265522,-0.0146377023592479,-0.0262522856574784,-0.0332982883404800,-0.0329785813529786,-0.0234889025494915,-0.00446795022720651,0.0227987111313552,0.0554733935803309,0.0895574698627017,0.120515653986008,0.144023869951959,0.156703240541716,0.156703240541716,0.144023869951959,0.120515653986008,0.0895574698627017,0.0554733935803309,0.0227987111313552,-0.00446795022720651,-0.0234889025494915,-0.0329785813529786,-0.0332982883404800,-0.0262522856574784,-0.0146377023592479,-0.00164793135265522,0.00974957079401820,0.0173392600079425,0.0199936468732958,0.0177664824921516,0.0117453379754635,0.00370978630578411,-0.00431738279400548,-0.0105285523000909,-0.0137001391812507,-0.0134041680526106,-0.0100287408975305,-0.00462116475466275,0.00140348734599666,0.00660729642869476,0.00985833280468664,0.0105569693922753,0.00873707931656989,0.00500716542551709,0.000370694735887264,-0.00402945969193484,-0.00718474600611743,-0.00844434474260682,-0.00763832593122044,-0.00508916383265880,-0.00150344842508819,0.00221406259836600,0.00518482450401190,0.00675694317361097,0.00664450212322939,0.00497204473913865,0.00222293200660433,-0.000895334632684916,-0.00362800246910812,-0.00535361761996745,-0.00572072307742373,-0.00471677657681364,-0.00265169080000238,-6.92633871522469e-05,0.00239414379788477,0.00416296073858980,0.00485590227040089,0.00436434760724723,0.00286531730560341,0.000763817543823744,-0.00141269996253721,-0.00314619916775268,-0.00404839890491124,-0.00394736724813684,-0.00291615727095717,-0.00124494075125038,0.000639685555779194,0.00228075576371429,0.00330196592411329,0.00349112151810480,0.00284398455796397,0.00155380296687732,-4.24245814635350e-05,-0.00155212468821863,-0.00262130441133404,-0.00301808363129534,-0.00268014785762408,-0.00172315033598271,-0.000403203461952063,0.000948917484967274,0.00201148722794215,0.00254612673517478,0.00245160754544550,0.00178015334256871,0.000719168292560208,-0.000461466156592793,-0.00147560705208410,-0.00209104798819840,-0.00218106334318668,-0.00174897886080622,-0.000924986301950368,8.07346505198323e-05,0.00101556569271055,0.00166463397417807,0.00188853225913880,0.00165203444124591,0.00103578588223087,0.000205480435592108,-0.000631676635553281,-0.00127789359379036,-0.00158898492169210,-0.00150778620717139,-0.00107300150830799,-0.000406447531171399,0.000322183468886171,0.000936422436097352,0.00129735552689972,0.00133223670076952,0.00105026034065052,0.000534226190937332,-8.07994050962511e-05,-0.000644283517924812,-0.00102480015587528,-0.00114242756498996,-0.000982925059372773,-0.000599851214765325,-9.57721550088415e-05,0.000402617249195520,0.000778164443560065,0.000948313052515260,0.000884953858656829,0.000615835391598136,0.000217061369202846,-0.000210083087778465,-0.000562283100847654,-0.000761179565529975,-0.000768131381123457,-0.000593492321711194,-0.000288951840645289,6.49334891517613e-05,0.000381973784588710,0.000588844945912514,0.000644104835925005,0.000543595791653537,0.000321661726173334,3.80197801358169e-05,-0.000235647756792349,-0.000435972930478074,-0.000519910225057264,-0.000476023611671503,-0.000322747046191756,-0.000104469482676730,0.000123551936096403,0.000306018107911122,0.000403778285772824,0.000399146152793842,0.000301322189111397,0.000139638083216722,-4.23373398720898e-05,-0.000203831166464738,-0.000227768927611145,-0.00207925902605286,-8.45011304875402e-05};
//float aaFilterBuffer[300] = {0};
//float usFilterBuffer[100] = {0};
//int resampleCnt = 0;
//int aaCnt = 0;
//int usCnt = 0;
//
//float window[512] = {0.000000e+00,3.913894e-03,7.827789e-03,1.174168e-02,1.565558e-02,1.956947e-02,2.348337e-02,2.739726e-02,3.131115e-02,3.522505e-02,3.913894e-02,4.305284e-02,4.696673e-02,5.088063e-02,5.479452e-02,5.870841e-02,6.262231e-02,6.653620e-02,7.045010e-02,7.436399e-02,7.827789e-02,8.219178e-02,8.610568e-02,9.001957e-02,9.393346e-02,9.784736e-02,1.017613e-01,1.056751e-01,1.095890e-01,1.135029e-01,1.174168e-01,1.213307e-01,1.252446e-01,1.291585e-01,1.330724e-01,1.369863e-01,1.409002e-01,1.448141e-01,1.487280e-01,1.526419e-01,1.565558e-01,1.604697e-01,1.643836e-01,1.682975e-01,1.722114e-01,1.761252e-01,1.800391e-01,1.839530e-01,1.878669e-01,1.917808e-01,1.956947e-01,1.996086e-01,2.035225e-01,2.074364e-01,2.113503e-01,2.152642e-01,2.191781e-01,2.230920e-01,2.270059e-01,2.309198e-01,2.348337e-01,2.387476e-01,2.426614e-01,2.465753e-01,2.504892e-01,2.544031e-01,2.583170e-01,2.622309e-01,2.661448e-01,2.700587e-01,2.739726e-01,2.778865e-01,2.818004e-01,2.857143e-01,2.896282e-01,2.935421e-01,2.974560e-01,3.013699e-01,3.052838e-01,3.091977e-01,3.131115e-01,3.170254e-01,3.209393e-01,3.248532e-01,3.287671e-01,3.326810e-01,3.365949e-01,3.405088e-01,3.444227e-01,3.483366e-01,3.522505e-01,3.561644e-01,3.600783e-01,3.639922e-01,3.679061e-01,3.718200e-01,3.757339e-01,3.796477e-01,3.835616e-01,3.874755e-01,3.913894e-01,3.953033e-01,3.992172e-01,4.031311e-01,4.070450e-01,4.109589e-01,4.148728e-01,4.187867e-01,4.227006e-01,4.266145e-01,4.305284e-01,4.344423e-01,4.383562e-01,4.422701e-01,4.461840e-01,4.500978e-01,4.540117e-01,4.579256e-01,4.618395e-01,4.657534e-01,4.696673e-01,4.735812e-01,4.774951e-01,4.814090e-01,4.853229e-01,4.892368e-01,4.931507e-01,4.970646e-01,5.009785e-01,5.048924e-01,5.088063e-01,5.127202e-01,5.166341e-01,5.205479e-01,5.244618e-01,5.283757e-01,5.322896e-01,5.362035e-01,5.401174e-01,5.440313e-01,5.479452e-01,5.518591e-01,5.557730e-01,5.596869e-01,5.636008e-01,5.675147e-01,5.714286e-01,5.753425e-01,5.792564e-01,5.831703e-01,5.870841e-01,5.909980e-01,5.949119e-01,5.988258e-01,6.027397e-01,6.066536e-01,6.105675e-01,6.144814e-01,6.183953e-01,6.223092e-01,6.262231e-01,6.301370e-01,6.340509e-01,6.379648e-01,6.418787e-01,6.457926e-01,6.497065e-01,6.536204e-01,6.575342e-01,6.614481e-01,6.653620e-01,6.692759e-01,6.731898e-01,6.771037e-01,6.810176e-01,6.849315e-01,6.888454e-01,6.927593e-01,6.966732e-01,7.005871e-01,7.045010e-01,7.084149e-01,7.123288e-01,7.162427e-01,7.201566e-01,7.240705e-01,7.279843e-01,7.318982e-01,7.358121e-01,7.397260e-01,7.436399e-01,7.475538e-01,7.514677e-01,7.553816e-01,7.592955e-01,7.632094e-01,7.671233e-01,7.710372e-01,7.749511e-01,7.788650e-01,7.827789e-01,7.866928e-01,7.906067e-01,7.945205e-01,7.984344e-01,8.023483e-01,8.062622e-01,8.101761e-01,8.140900e-01,8.180039e-01,8.219178e-01,8.258317e-01,8.297456e-01,8.336595e-01,8.375734e-01,8.414873e-01,8.454012e-01,8.493151e-01,8.532290e-01,8.571429e-01,8.610568e-01,8.649706e-01,8.688845e-01,8.727984e-01,8.767123e-01,8.806262e-01,8.845401e-01,8.884540e-01,8.923679e-01,8.962818e-01,9.001957e-01,9.041096e-01,9.080235e-01,9.119374e-01,9.158513e-01,9.197652e-01,9.236791e-01,9.275930e-01,9.315068e-01,9.354207e-01,9.393346e-01,9.432485e-01,9.471624e-01,9.510763e-01,9.549902e-01,9.589041e-01,9.628180e-01,9.667319e-01,9.706458e-01,9.745597e-01,9.784736e-01,9.823875e-01,9.863014e-01,9.902153e-01,9.941292e-01,9.980431e-01,9.980431e-01,9.941292e-01,9.902153e-01,9.863014e-01,9.823875e-01,9.784736e-01,9.745597e-01,9.706458e-01,9.667319e-01,9.628180e-01,9.589041e-01,9.549902e-01,9.510763e-01,9.471624e-01,9.432485e-01,9.393346e-01,9.354207e-01,9.315068e-01,9.275930e-01,9.236791e-01,9.197652e-01,9.158513e-01,9.119374e-01,9.080235e-01,9.041096e-01,9.001957e-01,8.962818e-01,8.923679e-01,8.884540e-01,8.845401e-01,8.806262e-01,8.767123e-01,8.727984e-01,8.688845e-01,8.649706e-01,8.610568e-01,8.571429e-01,8.532290e-01,8.493151e-01,8.454012e-01,8.414873e-01,8.375734e-01,8.336595e-01,8.297456e-01,8.258317e-01,8.219178e-01,8.180039e-01,8.140900e-01,8.101761e-01,8.062622e-01,8.023483e-01,7.984344e-01,7.945205e-01,7.906067e-01,7.866928e-01,7.827789e-01,7.788650e-01,7.749511e-01,7.710372e-01,7.671233e-01,7.632094e-01,7.592955e-01,7.553816e-01,7.514677e-01,7.475538e-01,7.436399e-01,7.397260e-01,7.358121e-01,7.318982e-01,7.279843e-01,7.240705e-01,7.201566e-01,7.162427e-01,7.123288e-01,7.084149e-01,7.045010e-01,7.005871e-01,6.966732e-01,6.927593e-01,6.888454e-01,6.849315e-01,6.810176e-01,6.771037e-01,6.731898e-01,6.692759e-01,6.653620e-01,6.614481e-01,6.575342e-01,6.536204e-01,6.497065e-01,6.457926e-01,6.418787e-01,6.379648e-01,6.340509e-01,6.301370e-01,6.262231e-01,6.223092e-01,6.183953e-01,6.144814e-01,6.105675e-01,6.066536e-01,6.027397e-01,5.988258e-01,5.949119e-01,5.909980e-01,5.870841e-01,5.831703e-01,5.792564e-01,5.753425e-01,5.714286e-01,5.675147e-01,5.636008e-01,5.596869e-01,5.557730e-01,5.518591e-01,5.479452e-01,5.440313e-01,5.401174e-01,5.362035e-01,5.322896e-01,5.283757e-01,5.244618e-01,5.205479e-01,5.166341e-01,5.127202e-01,5.088063e-01,5.048924e-01,5.009785e-01,4.970646e-01,4.931507e-01,4.892368e-01,4.853229e-01,4.814090e-01,4.774951e-01,4.735812e-01,4.696673e-01,4.657534e-01,4.618395e-01,4.579256e-01,4.540117e-01,4.500978e-01,4.461840e-01,4.422701e-01,4.383562e-01,4.344423e-01,4.305284e-01,4.266145e-01,4.227006e-01,4.187867e-01,4.148728e-01,4.109589e-01,4.070450e-01,4.031311e-01,3.992172e-01,3.953033e-01,3.913894e-01,3.874755e-01,3.835616e-01,3.796477e-01,3.757339e-01,3.718200e-01,3.679061e-01,3.639922e-01,3.600783e-01,3.561644e-01,3.522505e-01,3.483366e-01,3.444227e-01,3.405088e-01,3.365949e-01,3.326810e-01,3.287671e-01,3.248532e-01,3.209393e-01,3.170254e-01,3.131115e-01,3.091977e-01,3.052838e-01,3.013699e-01,2.974560e-01,2.935421e-01,2.896282e-01,2.857143e-01,2.818004e-01,2.778865e-01,2.739726e-01,2.700587e-01,2.661448e-01,2.622309e-01,2.583170e-01,2.544031e-01,2.504892e-01,2.465753e-01,2.426614e-01,2.387476e-01,2.348337e-01,2.309198e-01,2.270059e-01,2.230920e-01,2.191781e-01,2.152642e-01,2.113503e-01,2.074364e-01,2.035225e-01,1.996086e-01,1.956947e-01,1.917808e-01,1.878669e-01,1.839530e-01,1.800391e-01,1.761252e-01,1.722114e-01,1.682975e-01,1.643836e-01,1.604697e-01,1.565558e-01,1.526419e-01,1.487280e-01,1.448141e-01,1.409002e-01,1.369863e-01,1.330724e-01,1.291585e-01,1.252446e-01,1.213307e-01,1.174168e-01,1.135029e-01,1.095890e-01,1.056751e-01,1.017613e-01,9.784736e-02,9.393346e-02,9.001957e-02,8.610568e-02,8.219178e-02,7.827789e-02,7.436399e-02,7.045010e-02,6.653620e-02,6.262231e-02,5.870841e-02,5.479452e-02,5.088063e-02,4.696673e-02,4.305284e-02,3.913894e-02,3.522505e-02,3.131115e-02,2.739726e-02,2.348337e-02,1.956947e-02,1.565558e-02,1.174168e-02,7.827789e-03,3.913894e-03,0.000000e+00};
//float inputBuffer[512] = {0};
//float sigdata[512] = {0};
//int ioCnt = 0;
//float smoothWin[37] = {0,0.000422006860771999,0.00167520497816921,0.00372151656154337,0.00649876546891728,0.00992256639759613,0.0138888888888889,0.0182772182409536,0.0229542172870297,0.0277777777777778,0.0326013382685258,0.0372783373146019,0.0416666666666667,0.0456329891579594,0.0490567900866383,0.0518340389940122,0.0538803505773863,0.0551335486947836,0.0555555555555555,0.0551335486947836,0.0538803505773863,0.0518340389940122,0.0490567900866383,0.0456329891579594,0.0416666666666667,0.0372783373146019,0.0326013382685258,0.0277777777777778,0.0229542172870297,0.0182772182409536,0.0138888888888889,0.00992256639759613,0.00649876546891728,0.00372151656154337,0.00167520497816921,0.000422006860771999,0};
//
//complex_float ifftOut[512] = {0};
//
//complex_float inputFFT[512];
//complex_float outputFFT[512];
//
//complex_float i_temp[512];
//complex_float c_temp[512];
//float *r_temp = (float *) c_temp;
//complex_float pm twiddle_table[256];
//
//float outputArea[2][256] = {0};
//int outputAreaCnt = 0;
//
//float currentPower;
//float minimumPower = 1e32;
//float localMinimumPower = 1e32;
//float powerSpectrum[256] = {0};
//float noisePower[256] = {0};
//float localSpectrum[256] = {0};
//float Gain[257] = {0};
//
//int updCnt = 0;


/*

//ANC
#define SYS_LENGTH 	768
#define C_LENGTH	500


float reference;
float error;
float snr;

float control[C_LENGTH] = {-0.000104323600000000,-0.000206940500000000,-9.26465000000000e-05,-0.000114848600000000,-0.000109182500000000,-0.000105959200000000,-6.15507000000000e-05,-0.000203615100000000,-0.000158764100000000,-0.000163484200000000,-0.000244343500000000,-0.000203154200000000,-0.000209624400000000,-0.000312982700000000,-0.000461120600000000,-0.000177330000000000,-0.000571861800000000,0.000695945200000000,-0.00176668380000000,0.00252944590000000,-0.00439031950000000,0.00671408410000000,-0.0122556094000000,0.0685764167000000,0.152054119800000,0.333124382600000,0.157139152500000,-0.182941170600000,0.0340085693000000,-0.0238681448000000,-0.0949403445000000,-0.235656533700000,-0.0141081579000000,-0.0532379799000000,-0.0522247072000000,-0.126457602100000,0.0186776815000000,-0.0472564720000000,-0.0643912549000000,-0.0145752947000000,0.00376950210000000,0.00112456460000000,-0.0276768820000000,0.0139528955000000,-0.0326952079000000,0.0420818007000000,-0.0356303396000000,0.00631314800000000,-0.0210855962000000,-0.0387852147000000,-0.0175006184000000,-0.0182149075000000,-0.00667577630000000,-0.0102709498000000,-0.0277923535000000,-0.0246142608000000,0.0104832348000000,-0.0260501192000000,0.00955531990000000,0.0150214376000000,0.00971298790000000,0.0308739342000000,0.000944058000000000,0.0166392315000000,0.0121448468000000,-0.00768468400000000,-0.00362455970000000,0.00510981480000000,-0.00351265550000000,0.0130923502000000,0.0194884853000000,0.0153577066000000,0.0219752784000000,0.0293983827000000,0.00267366970000000,0.0149178131000000,0.0126271156000000,0.0162198040000000,0.0190786326000000,0.0174584054000000,0.0225625082000000,0.0179752478000000,0.0179876696000000,0.00996621960000000,0.0153553099000000,0.0123823709000000,0.0185168129000000,0.0221579651000000,0.0215953513000000,0.0240458162000000,0.0141716840000000,0.00994762990000000,0.0112961431000000,0.0107408267000000,0.00855447830000000,0.00707830780000000,0.0115910997000000,0.0106437132000000,0.0120421345000000,0.000799735700000000,0.00351625540000000,0.00483601700000000,0.00657386090000000,0.00626280910000000,0.00943047390000000,0.0102517914000000,0.00261407960000000,0.00221780590000000,0.00136008390000000,-0.00147758570000000,0.000884272700000000,0.00148133790000000,0.00142623590000000,-0.000628964200000000,0.00150901850000000,-0.00258079630000000,-0.00324987570000000,-0.00445161430000000,-0.00450014940000000,-0.00592393370000000,-0.00581484430000000,-0.00451103410000000,0.00960476260000000,0.0230872739000000,0.0168113169000000,-0.00997242970000000,-0.0249834865000000,-0.0105312971000000,-0.0139959076000000,-0.00908901250000000,-0.0127294643000000,-0.0268185389000000,-0.00769003320000000,-0.0225311222000000,-0.0196002271000000,-0.0147808364000000,-0.0212441561000000,-0.0225542864000000,-0.0186321634000000,-0.0199560285000000,-0.0123703576000000,-0.0127765898000000,-0.00583846770000000,-0.00559267840000000,-0.0148869133000000,-0.00705377040000000,-0.0143449757000000,-0.0117446210000000,-0.00788399720000000,-0.00812205320000000,-0.00875619020000000,-0.00387917430000000,-0.0115700368000000,-0.00694016640000000,-0.0101459759000000,-0.00845273040000000,-0.00707629430000000,-0.00672567140000000,-0.00787278570000000,-0.00470038110000000,-0.00185056400000000,-0.00253256370000000,-0.00411722030000000,-0.00948409180000000,-0.00815627940000000,-0.00678328800000000,-0.00302353780000000,-0.00482448600000000,0.000697058900000000,-0.00563214110000000,-0.000919733800000000,0.00657902030000000,0.00561873140000000,0.00363736110000000,-0.00429237530000000,0.00744917620000000,0.00620872570000000,-0.00207310430000000,0.00441097860000000,0.00411986680000000,0.00955582410000000,0.0148603633000000,0.0106072481000000,0.0137536786000000,0.00719229640000000,0.00560351380000000,-0.00477863160000000,0.00714602380000000,0.00484181210000000,-0.000907463800000000,0.000987190800000000,0.00360673080000000,0.00198913810000000,0.00943507550000000,0.00924138750000000,-0.00259406560000000,-0.00684275380000000,0.0133966308000000,0.00856572520000000,-0.000329542100000000,0.00779752810000000,0.00898663390000000,0.00711507280000000,0.0117461429000000,0.0131199794000000,-0.000375107700000000,-0.00142042400000000,0.0197732841000000,0.0129568240000000,-0.00713368140000000,-0.000912975100000000,-0.000977220800000000,-0.00453056760000000,0.00851162330000000,-0.00125145330000000,0.00152929690000000,0.00894672650000000,-0.000504792100000000,0.00355925840000000,-0.00158705180000000,-0.00218365510000000,0.00521403750000000,0.00106832680000000,-0.00262008040000000,0.00287119290000000,0.00524970350000000,0.00678983880000000,0.000961297600000000,-0.00187314970000000,0.00262904130000000,-0.000947320700000000,-0.00564196460000000,-0.00430944220000000,0.00489537970000000,0.0155295497000000,-0.00214058960000000,0.00685845380000000,0.00701305540000000,0.00105985930000000,0.00262721660000000,-0.00852518510000000,-0.0129708795000000,-0.0119423385000000,-0.00815426930000000,0.00142409510000000,-0.00804836050000000,-0.000877988200000000,0.00449650930000000,-0.00144676420000000,-0.00446723780000000,-0.00242831740000000,-0.00461090800000000,-0.0113920002000000,0.000822551900000000,-0.00552341750000000,-0.0103284891000000,-0.00662354260000000,-0.00437257980000000,-0.00767762080000000,0.000168930400000000,-0.00244701400000000,0.00103109930000000,0.00216614360000000,0.000345460900000000,0.00420870440000000,0.00347942160000000,-0.0101408151000000,-0.00475822660000000,-0.00466715580000000,-0.0166106227000000,-0.0119414028000000,-0.00368271370000000,0.00952617340000000,0.0106476407000000,0.0112586724000000,-0.0102499510000000,-0.0263391648000000,-0.00756535190000000,-0.00455714020000000,-0.00449821910000000,-0.00859606270000000,-0.0166431587000000,-0.00945278060000000,-0.00411695910000000,-0.00153357950000000,0.00291608540000000,-0.00147429090000000,0.0160896746000000,0.00504970200000000,0.00775229440000000,0.0128089563000000,0.0102159119000000,0.00221425450000000,0.00299515310000000,0.00556306690000000,0.00874506800000000,0.00130641470000000,-0.00166891370000000,0.000855908200000000,-0.00848451510000000,0.00687560450000000,-0.00680153270000000,0.00852585630000000,0.0112515903000000,-0.000480752400000000,-0.00139076720000000,-0.00835260040000000,-0.0110331481000000,-0.00783384070000000,-0.00379961480000000,0.00149781970000000,0.00148138790000000,0.000818432000000000,0.00651671810000000,0.00215728710000000,0.00375966260000000,0.00533864380000000,0.00532913890000000,0.00822323890000000,0.00111933600000000,0.00144677590000000,0.00252401120000000,0.00468628170000000,0.00390846330000000,0.00166128130000000,-0.00398570080000000,0.00440113010000000,-0.00127071810000000,-0.00781207160000000,-0.00560208220000000,0.00125944550000000,-0.0119836602000000,-0.00670698910000000,-0.00548881150000000,0.00368594950000000,-0.00299634570000000,-0.00138166550000000,-0.00502970880000000,-0.00638798880000000,0.0167957155000000,0.0143706980000000,0.0177450494000000,0.0195191014000000,0.00158307450000000,-0.00375265500000000,-0.00929733690000000,-0.00854125740000000,0.0113595532000000,0.0213013736000000,0.0200601967000000,0.0108289574000000,-0.00339551020000000,-0.00886273670000000,0.00260794580000000,-0.0147443487000000,-0.00726380160000000,0.00280612130000000,-0.00480022770000000,0.00556605610000000,-0.00828579900000000,-0.00368177260000000,0.00297923270000000,-0.00592114570000000,0.00276159110000000,0.00168159050000000,-0.00470388090000000,0.00112875280000000,-0.00282250790000000,0.00454951050000000,0.00187895240000000,-0.00781071530000000,-0.00170746540000000,0.00387373550000000,-0.00298584160000000,0.00145238190000000,3.31834000000000e-05,-0.00168058020000000,-0.00257819110000000,0.00383457610000000,-0.00169047680000000,0.00336249540000000,0.00334832970000000,0.00393921810000000,0.00216239920000000,-0.00348807960000000,-0.00156747490000000,0.000568934000000000,0.00365165750000000,-0.00128645030000000,-0.00420142780000000,-0.0109546220000000,-0.0139952850000000,-0.00438312100000000,-0.00476006660000000,-0.00567199710000000,0.00533862310000000,0.0111408531000000,0.0128750445000000,0.0114423879000000,0.0118995904000000,-0.000897565400000000,-0.00801454800000000,-0.00600218750000000,-0.00262355930000000,-0.00645873280000000,0.00192695350000000,0.00469884020000000,-0.00331175530000000,-0.000180341800000000,-0.000163910700000000,0.00401140270000000,0.00193154640000000,0.00538700250000000,0.00631996330000000,-0.00232213710000000,0.00186254750000000,-0.00147629620000000,-0.00659342330000000,-0.000913609500000000,0.00857209070000000,-0.000852165400000000,-0.00730641410000000,-0.00602081400000000,-0.0113149780000000,-0.00582717940000000,-0.00538922430000000,-0.00309733910000000,-0.0130878712000000,-0.00773537340000000,-0.00573110770000000,-0.00413029840000000,-0.00323906190000000,-0.00349208110000000,0.00595541080000000,-0.00556343880000000,-0.00502789680000000,-0.00836931030000000,-0.0207842101000000,-0.0188738455000000,-0.0100856932000000,-0.00926077590000000,-0.00533928150000000,-0.000396892200000000,0.0147217780000000,0.00875041330000000,0.00194450950000000,0.00297371120000000,0.00808842650000000,0.00141280310000000,0.00624972270000000,0.00599985100000000,-0.000405685400000000,0.00681743000000000,0.00520755710000000,0.00324769200000000,0.00387890950000000,-0.00290316930000000,0.000334956900000000,0.00335399510000000,0.00189242010000000,0.00426381370000000,0.00191774980000000,-0.00177779270000000,0.00105348720000000,0.00310479300000000,0.00692817830000000,0.0100358285000000,-0.00150178220000000,0.00699023970000000,0.00709644040000000,-0.00234421220000000,-0.00117997200000000,0.00306989470000000,0.00103963500000000,-0.00364817100000000,0.00101488470000000,0.00487117940000000,0.00159488910000000,0.00122252910000000,0.00776445590000000,0.00637464970000000,0.00687214220000000,0.00894628250000000,0.00736093710000000,-0.000286937400000000,0.00546570030000000,0.000621147600000000,-0.000237139800000000,-0.000319636600000000,0.000491321500000000,-0.00226395430000000,-0.00181769800000000,0.00144660370000000,-4.61556000000000e-05,0.000613385900000000,-0.00161616660000000,-0.00243726430000000,0.00194832210000000,0.00138407650000000,-0.00493820090000000};
float filter[SYS_LENGTH] = {0};

float x2cBuffer[C_LENGTH] = {0};
float yBuffer[C_LENGTH] = {0};
int controlCnt = 0;
float x2hBuffer[SYS_LENGTH] = {0};
float zBuffer[SYS_LENGTH] = {0};
float errorBuffer[SYS_LENGTH] = {0};
int filterCnt = 0;



int updCnt = 0;
int logCnt = 0;
float errorLog[500];

float delayBuffer[40] = {0};
int delayCnt = 0;
*/


//AEC
#define FILTER_LENGTH 3200
#define VAD_LENGTH 96

float outgoing;
float error;
float snr = 1;
float speakerBuffer[FILTER_LENGTH] = {0};
int converge = 1;

float filter[FILTER_LENGTH] = {0};
float filterBuffer[FILTER_LENGTH] = {0};
int filterCnt = 0;

float aaFilter[300] = {-8.45011304875402e-05,-0.00207925902605286,-0.000227768927611145,-0.000203831166464738,-4.23373398720898e-05,0.000139638083216722,0.000301322189111397,0.000399146152793842,0.000403778285772824,0.000306018107911122,0.000123551936096403,-0.000104469482676730,-0.000322747046191756,-0.000476023611671503,-0.000519910225057264,-0.000435972930478074,-0.000235647756792349,3.80197801358169e-05,0.000321661726173334,0.000543595791653537,0.000644104835925005,0.000588844945912514,0.000381973784588710,6.49334891517613e-05,-0.000288951840645289,-0.000593492321711194,-0.000768131381123457,-0.000761179565529975,-0.000562283100847654,-0.000210083087778465,0.000217061369202846,0.000615835391598136,0.000884953858656829,0.000948313052515260,0.000778164443560065,0.000402617249195520,-9.57721550088415e-05,-0.000599851214765325,-0.000982925059372773,-0.00114242756498996,-0.00102480015587528,-0.000644283517924812,-8.07994050962511e-05,0.000534226190937332,0.00105026034065052,0.00133223670076952,0.00129735552689972,0.000936422436097352,0.000322183468886171,-0.000406447531171399,-0.00107300150830799,-0.00150778620717139,-0.00158898492169210,-0.00127789359379036,-0.000631676635553281,0.000205480435592108,0.00103578588223087,0.00165203444124591,0.00188853225913880,0.00166463397417807,0.00101556569271055,8.07346505198323e-05,-0.000924986301950368,-0.00174897886080622,-0.00218106334318668,-0.00209104798819840,-0.00147560705208410,-0.000461466156592793,0.000719168292560208,0.00178015334256871,0.00245160754544550,0.00254612673517478,0.00201148722794215,0.000948917484967274,-0.000403203461952063,-0.00172315033598271,-0.00268014785762408,-0.00301808363129534,-0.00262130441133404,-0.00155212468821863,-4.24245814635350e-05,0.00155380296687732,0.00284398455796397,0.00349112151810480,0.00330196592411329,0.00228075576371429,0.000639685555779194,-0.00124494075125038,-0.00291615727095717,-0.00394736724813684,-0.00404839890491124,-0.00314619916775268,-0.00141269996253721,0.000763817543823744,0.00286531730560341,0.00436434760724723,0.00485590227040089,0.00416296073858980,0.00239414379788477,-6.92633871522469e-05,-0.00265169080000238,-0.00471677657681364,-0.00572072307742373,-0.00535361761996745,-0.00362800246910812,-0.000895334632684916,0.00222293200660433,0.00497204473913865,0.00664450212322939,0.00675694317361097,0.00518482450401190,0.00221406259836600,-0.00150344842508819,-0.00508916383265880,-0.00763832593122044,-0.00844434474260682,-0.00718474600611743,-0.00402945969193484,0.000370694735887264,0.00500716542551709,0.00873707931656989,0.0105569693922753,0.00985833280468664,0.00660729642869476,0.00140348734599666,-0.00462116475466275,-0.0100287408975305,-0.0134041680526106,-0.0137001391812507,-0.0105285523000909,-0.00431738279400548,0.00370978630578411,0.0117453379754635,0.0177664824921516,0.0199936468732958,0.0173392600079425,0.00974957079401820,-0.00164793135265522,-0.0146377023592479,-0.0262522856574784,-0.0332982883404800,-0.0329785813529786,-0.0234889025494915,-0.00446795022720651,0.0227987111313552,0.0554733935803309,0.0895574698627017,0.120515653986008,0.144023869951959,0.156703240541716,0.156703240541716,0.144023869951959,0.120515653986008,0.0895574698627017,0.0554733935803309,0.0227987111313552,-0.00446795022720651,-0.0234889025494915,-0.0329785813529786,-0.0332982883404800,-0.0262522856574784,-0.0146377023592479,-0.00164793135265522,0.00974957079401820,0.0173392600079425,0.0199936468732958,0.0177664824921516,0.0117453379754635,0.00370978630578411,-0.00431738279400548,-0.0105285523000909,-0.0137001391812507,-0.0134041680526106,-0.0100287408975305,-0.00462116475466275,0.00140348734599666,0.00660729642869476,0.00985833280468664,0.0105569693922753,0.00873707931656989,0.00500716542551709,0.000370694735887264,-0.00402945969193484,-0.00718474600611743,-0.00844434474260682,-0.00763832593122044,-0.00508916383265880,-0.00150344842508819,0.00221406259836600,0.00518482450401190,0.00675694317361097,0.00664450212322939,0.00497204473913865,0.00222293200660433,-0.000895334632684916,-0.00362800246910812,-0.00535361761996745,-0.00572072307742373,-0.00471677657681364,-0.00265169080000238,-6.92633871522469e-05,0.00239414379788477,0.00416296073858980,0.00485590227040089,0.00436434760724723,0.00286531730560341,0.000763817543823744,-0.00141269996253721,-0.00314619916775268,-0.00404839890491124,-0.00394736724813684,-0.00291615727095717,-0.00124494075125038,0.000639685555779194,0.00228075576371429,0.00330196592411329,0.00349112151810480,0.00284398455796397,0.00155380296687732,-4.24245814635350e-05,-0.00155212468821863,-0.00262130441133404,-0.00301808363129534,-0.00268014785762408,-0.00172315033598271,-0.000403203461952063,0.000948917484967274,0.00201148722794215,0.00254612673517478,0.00245160754544550,0.00178015334256871,0.000719168292560208,-0.000461466156592793,-0.00147560705208410,-0.00209104798819840,-0.00218106334318668,-0.00174897886080622,-0.000924986301950368,8.07346505198323e-05,0.00101556569271055,0.00166463397417807,0.00188853225913880,0.00165203444124591,0.00103578588223087,0.000205480435592108,-0.000631676635553281,-0.00127789359379036,-0.00158898492169210,-0.00150778620717139,-0.00107300150830799,-0.000406447531171399,0.000322183468886171,0.000936422436097352,0.00129735552689972,0.00133223670076952,0.00105026034065052,0.000534226190937332,-8.07994050962511e-05,-0.000644283517924812,-0.00102480015587528,-0.00114242756498996,-0.000982925059372773,-0.000599851214765325,-9.57721550088415e-05,0.000402617249195520,0.000778164443560065,0.000948313052515260,0.000884953858656829,0.000615835391598136,0.000217061369202846,-0.000210083087778465,-0.000562283100847654,-0.000761179565529975,-0.000768131381123457,-0.000593492321711194,-0.000288951840645289,6.49334891517613e-05,0.000381973784588710,0.000588844945912514,0.000644104835925005,0.000543595791653537,0.000321661726173334,3.80197801358169e-05,-0.000235647756792349,-0.000435972930478074,-0.000519910225057264,-0.000476023611671503,-0.000322747046191756,-0.000104469482676730,0.000123551936096403,0.000306018107911122,0.000403778285772824,0.000399146152793842,0.000301322189111397,0.000139638083216722,-4.23373398720898e-05,-0.000203831166464738,-0.000227768927611145,-0.00207925902605286,-8.45011304875402e-05};
float aaFilterBuffer0[300] = {0};
float usFilterBuffer0[100] = {0};
float aaFilterBuffer1[300] = {0};
float usFilterBuffer1[100] = {0};
int resampleCnt = 0;
int aaCnt = 0;
int usCnt = 0;

float VADbuffer0[VAD_LENGTH] = {0};
float VADbuffer1[VAD_LENGTH] = {0};
int VADcnt = 0;
int VAD = 1;
int pauseCnt = 0;

float window[512] = {0.000000e+00,3.913894e-03,7.827789e-03,1.174168e-02,1.565558e-02,1.956947e-02,2.348337e-02,2.739726e-02,3.131115e-02,3.522505e-02,3.913894e-02,4.305284e-02,4.696673e-02,5.088063e-02,5.479452e-02,5.870841e-02,6.262231e-02,6.653620e-02,7.045010e-02,7.436399e-02,7.827789e-02,8.219178e-02,8.610568e-02,9.001957e-02,9.393346e-02,9.784736e-02,1.017613e-01,1.056751e-01,1.095890e-01,1.135029e-01,1.174168e-01,1.213307e-01,1.252446e-01,1.291585e-01,1.330724e-01,1.369863e-01,1.409002e-01,1.448141e-01,1.487280e-01,1.526419e-01,1.565558e-01,1.604697e-01,1.643836e-01,1.682975e-01,1.722114e-01,1.761252e-01,1.800391e-01,1.839530e-01,1.878669e-01,1.917808e-01,1.956947e-01,1.996086e-01,2.035225e-01,2.074364e-01,2.113503e-01,2.152642e-01,2.191781e-01,2.230920e-01,2.270059e-01,2.309198e-01,2.348337e-01,2.387476e-01,2.426614e-01,2.465753e-01,2.504892e-01,2.544031e-01,2.583170e-01,2.622309e-01,2.661448e-01,2.700587e-01,2.739726e-01,2.778865e-01,2.818004e-01,2.857143e-01,2.896282e-01,2.935421e-01,2.974560e-01,3.013699e-01,3.052838e-01,3.091977e-01,3.131115e-01,3.170254e-01,3.209393e-01,3.248532e-01,3.287671e-01,3.326810e-01,3.365949e-01,3.405088e-01,3.444227e-01,3.483366e-01,3.522505e-01,3.561644e-01,3.600783e-01,3.639922e-01,3.679061e-01,3.718200e-01,3.757339e-01,3.796477e-01,3.835616e-01,3.874755e-01,3.913894e-01,3.953033e-01,3.992172e-01,4.031311e-01,4.070450e-01,4.109589e-01,4.148728e-01,4.187867e-01,4.227006e-01,4.266145e-01,4.305284e-01,4.344423e-01,4.383562e-01,4.422701e-01,4.461840e-01,4.500978e-01,4.540117e-01,4.579256e-01,4.618395e-01,4.657534e-01,4.696673e-01,4.735812e-01,4.774951e-01,4.814090e-01,4.853229e-01,4.892368e-01,4.931507e-01,4.970646e-01,5.009785e-01,5.048924e-01,5.088063e-01,5.127202e-01,5.166341e-01,5.205479e-01,5.244618e-01,5.283757e-01,5.322896e-01,5.362035e-01,5.401174e-01,5.440313e-01,5.479452e-01,5.518591e-01,5.557730e-01,5.596869e-01,5.636008e-01,5.675147e-01,5.714286e-01,5.753425e-01,5.792564e-01,5.831703e-01,5.870841e-01,5.909980e-01,5.949119e-01,5.988258e-01,6.027397e-01,6.066536e-01,6.105675e-01,6.144814e-01,6.183953e-01,6.223092e-01,6.262231e-01,6.301370e-01,6.340509e-01,6.379648e-01,6.418787e-01,6.457926e-01,6.497065e-01,6.536204e-01,6.575342e-01,6.614481e-01,6.653620e-01,6.692759e-01,6.731898e-01,6.771037e-01,6.810176e-01,6.849315e-01,6.888454e-01,6.927593e-01,6.966732e-01,7.005871e-01,7.045010e-01,7.084149e-01,7.123288e-01,7.162427e-01,7.201566e-01,7.240705e-01,7.279843e-01,7.318982e-01,7.358121e-01,7.397260e-01,7.436399e-01,7.475538e-01,7.514677e-01,7.553816e-01,7.592955e-01,7.632094e-01,7.671233e-01,7.710372e-01,7.749511e-01,7.788650e-01,7.827789e-01,7.866928e-01,7.906067e-01,7.945205e-01,7.984344e-01,8.023483e-01,8.062622e-01,8.101761e-01,8.140900e-01,8.180039e-01,8.219178e-01,8.258317e-01,8.297456e-01,8.336595e-01,8.375734e-01,8.414873e-01,8.454012e-01,8.493151e-01,8.532290e-01,8.571429e-01,8.610568e-01,8.649706e-01,8.688845e-01,8.727984e-01,8.767123e-01,8.806262e-01,8.845401e-01,8.884540e-01,8.923679e-01,8.962818e-01,9.001957e-01,9.041096e-01,9.080235e-01,9.119374e-01,9.158513e-01,9.197652e-01,9.236791e-01,9.275930e-01,9.315068e-01,9.354207e-01,9.393346e-01,9.432485e-01,9.471624e-01,9.510763e-01,9.549902e-01,9.589041e-01,9.628180e-01,9.667319e-01,9.706458e-01,9.745597e-01,9.784736e-01,9.823875e-01,9.863014e-01,9.902153e-01,9.941292e-01,9.980431e-01,9.980431e-01,9.941292e-01,9.902153e-01,9.863014e-01,9.823875e-01,9.784736e-01,9.745597e-01,9.706458e-01,9.667319e-01,9.628180e-01,9.589041e-01,9.549902e-01,9.510763e-01,9.471624e-01,9.432485e-01,9.393346e-01,9.354207e-01,9.315068e-01,9.275930e-01,9.236791e-01,9.197652e-01,9.158513e-01,9.119374e-01,9.080235e-01,9.041096e-01,9.001957e-01,8.962818e-01,8.923679e-01,8.884540e-01,8.845401e-01,8.806262e-01,8.767123e-01,8.727984e-01,8.688845e-01,8.649706e-01,8.610568e-01,8.571429e-01,8.532290e-01,8.493151e-01,8.454012e-01,8.414873e-01,8.375734e-01,8.336595e-01,8.297456e-01,8.258317e-01,8.219178e-01,8.180039e-01,8.140900e-01,8.101761e-01,8.062622e-01,8.023483e-01,7.984344e-01,7.945205e-01,7.906067e-01,7.866928e-01,7.827789e-01,7.788650e-01,7.749511e-01,7.710372e-01,7.671233e-01,7.632094e-01,7.592955e-01,7.553816e-01,7.514677e-01,7.475538e-01,7.436399e-01,7.397260e-01,7.358121e-01,7.318982e-01,7.279843e-01,7.240705e-01,7.201566e-01,7.162427e-01,7.123288e-01,7.084149e-01,7.045010e-01,7.005871e-01,6.966732e-01,6.927593e-01,6.888454e-01,6.849315e-01,6.810176e-01,6.771037e-01,6.731898e-01,6.692759e-01,6.653620e-01,6.614481e-01,6.575342e-01,6.536204e-01,6.497065e-01,6.457926e-01,6.418787e-01,6.379648e-01,6.340509e-01,6.301370e-01,6.262231e-01,6.223092e-01,6.183953e-01,6.144814e-01,6.105675e-01,6.066536e-01,6.027397e-01,5.988258e-01,5.949119e-01,5.909980e-01,5.870841e-01,5.831703e-01,5.792564e-01,5.753425e-01,5.714286e-01,5.675147e-01,5.636008e-01,5.596869e-01,5.557730e-01,5.518591e-01,5.479452e-01,5.440313e-01,5.401174e-01,5.362035e-01,5.322896e-01,5.283757e-01,5.244618e-01,5.205479e-01,5.166341e-01,5.127202e-01,5.088063e-01,5.048924e-01,5.009785e-01,4.970646e-01,4.931507e-01,4.892368e-01,4.853229e-01,4.814090e-01,4.774951e-01,4.735812e-01,4.696673e-01,4.657534e-01,4.618395e-01,4.579256e-01,4.540117e-01,4.500978e-01,4.461840e-01,4.422701e-01,4.383562e-01,4.344423e-01,4.305284e-01,4.266145e-01,4.227006e-01,4.187867e-01,4.148728e-01,4.109589e-01,4.070450e-01,4.031311e-01,3.992172e-01,3.953033e-01,3.913894e-01,3.874755e-01,3.835616e-01,3.796477e-01,3.757339e-01,3.718200e-01,3.679061e-01,3.639922e-01,3.600783e-01,3.561644e-01,3.522505e-01,3.483366e-01,3.444227e-01,3.405088e-01,3.365949e-01,3.326810e-01,3.287671e-01,3.248532e-01,3.209393e-01,3.170254e-01,3.131115e-01,3.091977e-01,3.052838e-01,3.013699e-01,2.974560e-01,2.935421e-01,2.896282e-01,2.857143e-01,2.818004e-01,2.778865e-01,2.739726e-01,2.700587e-01,2.661448e-01,2.622309e-01,2.583170e-01,2.544031e-01,2.504892e-01,2.465753e-01,2.426614e-01,2.387476e-01,2.348337e-01,2.309198e-01,2.270059e-01,2.230920e-01,2.191781e-01,2.152642e-01,2.113503e-01,2.074364e-01,2.035225e-01,1.996086e-01,1.956947e-01,1.917808e-01,1.878669e-01,1.839530e-01,1.800391e-01,1.761252e-01,1.722114e-01,1.682975e-01,1.643836e-01,1.604697e-01,1.565558e-01,1.526419e-01,1.487280e-01,1.448141e-01,1.409002e-01,1.369863e-01,1.330724e-01,1.291585e-01,1.252446e-01,1.213307e-01,1.174168e-01,1.135029e-01,1.095890e-01,1.056751e-01,1.017613e-01,9.784736e-02,9.393346e-02,9.001957e-02,8.610568e-02,8.219178e-02,7.827789e-02,7.436399e-02,7.045010e-02,6.653620e-02,6.262231e-02,5.870841e-02,5.479452e-02,5.088063e-02,4.696673e-02,4.305284e-02,3.913894e-02,3.522505e-02,3.131115e-02,2.739726e-02,2.348337e-02,1.956947e-02,1.565558e-02,1.174168e-02,7.827789e-03,3.913894e-03,0.000000e+00};
float inputBuffer[512] = {0};
float sigdata[512] = {0};
int ioCnt = 0;
float smoothWin[37] = {0,0.000422006860771999,0.00167520497816921,0.00372151656154337,0.00649876546891728,0.00992256639759613,0.0138888888888889,0.0182772182409536,0.0229542172870297,0.0277777777777778,0.0326013382685258,0.0372783373146019,0.0416666666666667,0.0456329891579594,0.0490567900866383,0.0518340389940122,0.0538803505773863,0.0551335486947836,0.0555555555555555,0.0551335486947836,0.0538803505773863,0.0518340389940122,0.0490567900866383,0.0456329891579594,0.0416666666666667,0.0372783373146019,0.0326013382685258,0.0277777777777778,0.0229542172870297,0.0182772182409536,0.0138888888888889,0.00992256639759613,0.00649876546891728,0.00372151656154337,0.00167520497816921,0.000422006860771999,0};

complex_float ifftOut[512] = {0};

complex_float inputFFT[512];
complex_float outputFFT[512];

complex_float i_temp[512];
complex_float c_temp[512];
float *r_temp = (float *) c_temp;
complex_float pm twiddle_table[256];

float outputArea[2][256] = {0};
int outputAreaCnt = 0;

float currentPower;
float minimumPower = 1e32;
float localMinimumPower = 1e32;
float powerSpectrum[256] = {0};
float noisePower[256] = {0};
float localSpectrum[256] = {0};
float Gain[257] = {0};

int updCnt = 0;


uint32_t ProcessBuffers(void)
{
    ADI_ADAU1979_RESULT     eResult1;
	ADI_ADAU1962A_RESULT    eResult2;

	uint32_t i =0u;
	//int8_t *pSrc;
	//int8_t *pDst;
	int *pSrc;
	int *pDst;
	void *pDAC;
	twidfft (twiddle_table,512);

	if(pGetADC != NULL)
	{
		pADC = (void *)pGetADC;
		/* re-submit the ADC buffer */
		eResult1 = adi_adau1979_SubmitBuffer(phAdau1979, (void *) pADC, AUDIO_BUFFER_SIZE);
		if(eResult1)
		{
			return 1u;
		}
	    pGetADC = NULL;
	}

	if (pGetDAC != NULL)
	{
		pDAC = (void *)pGetDAC;
		/* re-submit the DAC buffer */
	    eResult2 = adi_adau1962a_SubmitBuffer(phAdau1962a, (void *) pDAC, AUDIO_BUFFER_SIZE);

		if(eResult2)
		{
			return 1u;
		}

	    pGetDAC = NULL;

	}

	/* copy ADC to DAC buffer */
	if ((pDAC != NULL) && (pADC != NULL))
	{
		//pSrc =  (int8_t *)pADC;
		//pDst =  (int8_t *)pDAC;
		pSrc =  (int *)pADC;
		pDst =  (int *)pDAC;
/* Channel Define
 * Ch2 Ch3
 * Ch0 Ch1
 */

#ifdef PLAYBACK
#ifndef TDM_MODE
		for(i=0; i<AUDIO_BUFFER_SIZE/NUM_CHANNELS;i+=4)
		{


//			inputBuffer[filterCnt] = (float)*(pSrc+i);
//			filterBuffer[filterCnt] = (float)*(pSrc+i+1);
//			error = Fir_Dir_I(FILTER_LENGTH,filterCnt,filter,filterBuffer) + (float)*(pSrc+i);
//			outputCum[filterCnt] = error;
//			*(pDst+i) = error;
//			*(pDst+i+1) = error;
//
//			if(filterCnt==(FILTER_LENGTH-1))
//			{
//				snr = vector_norm(inputBuffer,FILTER_LENGTH)/vector_norm(filterBuffer,FILTER_LENGTH);
//			}
//			if(snr < 1)
//			{
//				filter_upd(FILTER_LENGTH,filterCnt,filter,filterBuffer,error);
//			}
//
//			filterCnt = (filterCnt + 1) % FILTER_LENGTH;
//
			aaFilterBuffer0[aaCnt] = (float)*(pSrc+i);
			aaFilterBuffer1[aaCnt] = (float)*(pSrc+i+1);
			if (resampleCnt==0)
			{
				speakerBuffer[filterCnt] = Fir_Dir_I(300,aaCnt,aaFilter,aaFilterBuffer0);
				filterBuffer[filterCnt] = Fir_Dir_I(300,aaCnt,aaFilter,aaFilterBuffer1);
				usFilterBuffer0[usCnt] = outputArea[outputAreaCnt][ioCnt] * 3;
				error = Fir_Dir_I(FILTER_LENGTH,filterCnt,filter,filterBuffer) + speakerBuffer[filterCnt];

				//Spectral Subtract Denoise
				inputBuffer[256 + ioCnt] = error;
				sigdata[ioCnt] = inputBuffer[ioCnt] * window[ioCnt];
				sigdata[256 + ioCnt] = inputBuffer[256 + ioCnt] * window[256 + ioCnt];

				if(ioCnt==255)
				{
					currentPower = vector_norm(inputBuffer,512);
					rfft(sigdata,r_temp,inputFFT,twiddle_table,1,512);
					ps_update(powerSpectrum,inputFFT);
					if(currentPower < minimumPower)
					{
						array_copy(powerSpectrum,noisePower);
						localMinimumPower = 1e32;
						minimumPower = currentPower;
						updCnt = 0;
					}
					else if((currentPower<localMinimumPower)&(updCnt<255))
					{
						localMinimumPower = currentPower;
						array_copy(powerSpectrum,localSpectrum);
						updCnt++;
					}
					else if(updCnt==255)
					{
						array_copy(localSpectrum,noisePower);
						minimumPower = localMinimumPower;
						localMinimumPower = 1e32;
						updCnt = 0;
					}
					else
					{
						updCnt++;
					}

					noise_reduction(noisePower, powerSpectrum, inputFFT, outputFFT);

					for(int i = 1; i < 256; i++)
					{
						outputFFT[512 - i] = conjf(outputFFT[i]);
					}
					ifft (outputFFT,i_temp,ifftOut,twiddle_table,1,512);

					int temp = (outputAreaCnt + 1) % 2;
					for(int i = 0; i < 256; i++)
					{
						outputArea[temp][i] = outputArea[temp][i] + ifftOut[i].re;
						outputArea[outputAreaCnt][i] = ifftOut[256 + i].re;
						inputBuffer[i] = inputBuffer[256 + i];
					}
					outputAreaCnt = temp;
				}

				ioCnt = (ioCnt + 1) % 256;

				//VAD and AEC filter update
				VADbuffer0[VADcnt] = usFilterBuffer0[usCnt];
				VADbuffer1[VADcnt] = filterBuffer[filterCnt];
				if(VADcnt==(VAD_LENGTH - 1))
				{
					if(vector_norm(VADbuffer0,VAD_LENGTH)<5*1e12)
					{
						if(pauseCnt==32&vector_norm(VADbuffer1,VAD_LENGTH)>1e12)
							VAD = 1;
						else if(pauseCnt<32)
							pauseCnt++;
					}
					else
					{
						pauseCnt = 0;
						VAD = 0;
					}
				}
				if(VAD == 1)
				{
					filter_upd(FILTER_LENGTH,filterCnt,filter,filterBuffer,error);
				}

				filterCnt = (filterCnt + 1) % FILTER_LENGTH;

				VADcnt = (VADcnt + 1) % VAD_LENGTH;
				usCnt = (usCnt + 1) % 100;
			}
			outgoing =  Upsampling_FIR(300,usCnt,resampleCnt,aaFilter,usFilterBuffer0);
			*(pDst+i) = *(pSrc+i);
			*(pDst+i+1) = outgoing;
			resampleCnt = (resampleCnt + 1) % 3;
			aaCnt = (aaCnt + 1) % 300;




/*
			//ANC
			reference = (float)*(pSrc+i);
			reference = reference/8388607;
			x2hBuffer[filterCnt] = reference;
			x2cBuffer[controlCnt] = reference;



//			zBuffer[filterCnt] = x2cBuffer[controlCnt];
//			error = delayBuffer[delayCnt];
//			error = error + Fir_Dir_I(SYS_LENGTH,filterCnt,filter,x2hBuffer);

			yBuffer[controlCnt] = Fir_Dir_I(SYS_LENGTH,filterCnt,filter,x2hBuffer);
			zBuffer[filterCnt] = Fir_Dir_I(C_LENGTH,controlCnt,control,x2cBuffer);
			error=delayBuffer[delayCnt];
			error = (float)*(pSrc+i+1);
			error = error/8388607;
			error = error + Fir_Dir_I(C_LENGTH,controlCnt,control,yBuffer);
			errorBuffer[filterCnt] = error;


			if(filterCnt==SYS_LENGTH - 1)
			{
				snr = vector_norm(errorBuffer,SYS_LENGTH)/(vector_norm(x2hBuffer,SYS_LENGTH)+1e-20);

			}
			if(snr>0.01)
				filter_upd(SYS_LENGTH,filterCnt,filter,zBuffer,error);

			if((updCnt==479)&(logCnt<500))
			{
					errorLog[logCnt] = snr;
				logCnt++;
			}
			*(pDst+i) = (int)(error*8388607);
			*(pDst+i+1) = *(pSrc+i);

			delayBuffer[delayCnt] = reference;
			delayCnt = (delayCnt + 1) % 40;
			updCnt = (updCnt + 1) % 480;
			filterCnt = (filterCnt + 1) % SYS_LENGTH;
			controlCnt = (controlCnt + 1) % C_LENGTH;

*/





//			aaFilterBuffer[aaCnt] = (float)*(pSrc+i);
//			if (resampleCnt==0)
//			{
//				inputBuffer[256 + ioCnt] = Fir_Dir_I(300,aaCnt,aaFilter,aaFilterBuffer);
//				sigdata[ioCnt] = inputBuffer[ioCnt] * window[ioCnt];
//				sigdata[256 + ioCnt] = inputBuffer[256 + ioCnt] * window[256 + ioCnt];
//				usFilterBuffer[usCnt] = outputArea[outputAreaCnt][ioCnt] * 3;
//
//				if(ioCnt==255)
//				{
//					currentPower = vector_norm(inputBuffer,512);
//					rfft(sigdata,r_temp,inputFFT,twiddle_table,1,512);
//					ps_update(powerSpectrum,inputFFT);
//					if(currentPower < minimumPower)
//					{
//						array_copy(powerSpectrum,noisePower);
//						localMinimumPower = 1e32;
//						minimumPower = currentPower;
//						updCnt = 0;
//					}
//					else if((currentPower<localMinimumPower)&(updCnt<255))
//					{
//						localMinimumPower = currentPower;
//						array_copy(powerSpectrum,localSpectrum);
//						updCnt++;
//					}
//					else if(updCnt==255)
//					{
//						array_copy(localSpectrum,noisePower);
//						minimumPower = localMinimumPower;
//						localMinimumPower = 1e32;
//						updCnt = 0;
//					}
//					else
//					{
//						updCnt++;
//					}
//
//					noise_reduction(noisePower, powerSpectrum, inputFFT, outputFFT);
//
//					for(int i = 1; i < 256; i++)
//					{
//						outputFFT[512 - i] = conjf(outputFFT[i]);
//					}
//					ifft (outputFFT,i_temp,ifftOut,twiddle_table,1,512);
//
//					int temp = (outputAreaCnt + 1) % 2;
//					for(int i = 0; i < 256; i++)
//					{
//						outputArea[temp][i] = outputArea[temp][i] + ifftOut[i].re;
//						outputArea[outputAreaCnt][i] = ifftOut[256 + i].re;
//						inputBuffer[i] = inputBuffer[256 + i];
//					}
//					outputAreaCnt = temp;
//				}
//
//				ioCnt = (ioCnt + 1) % 256;
//				usCnt = (usCnt + 1) % 100;
//			}
//
//			*(pDst+i) =  (int)Upsampling_FIR(300,usCnt,resampleCnt,aaFilter,usFilterBuffer);
//			resampleCnt = (resampleCnt + 1) % 3;
//			aaCnt = (aaCnt + 1) % 300;
//
//			*(pDst+i+1) = *(pSrc+i+1);


//			if(capDelay<CAP_DELAY)
//				capDelay++;
//			else if(capCnt<SYS_LENGTH)
//			{
//				xCap[capCnt] = (float)*(pSrc+i+2) - temp[0];
//				yCap[capCnt] = (float)*(pSrc+i+3) - temp[1];
//				temp[0] = (float)*(pSrc+i+2);
//				temp[1] = (float)*(pSrc+i+3);
////				xCap[capCnt] = (float)*(pSrc+i+2);
////				yCap[capCnt] = (float)*(pSrc+i+3);
//				capCnt++;
//			}
//			else if(capCnt==SYS_LENGTH)
//			{
//				error = yCap[SYS_LENGTH - 1] - Fir_Dir_I(SYS_LENGTH,SYS_LENGTH - 1,firFit,xCap);
////				convergence[epoch] = error * error;
//				if((error<(1e01)&error>-(1e01))||epoch == 30000)
//				{
//					printf("Done at epoch %d.\n",epoch);
//					printf("Final error: %e.\n",error);
//					capCnt++;
//				}
//				else
//				{
//					NLMSSGD(xCap,firFit,error,SYS_LENGTH,STEP_SIZE);
//					capCnt = 0;
//					epoch++;
//				}
//			}
//
//
//			output = (float)(rand());
//			output = output / RAND_MAX * 2 - 1;
//			firBuffer[firBufferCounter]=output;
//
//			*(pDst+i+2) = (int)(output * 8388607.0f);
//			*(pDst+i+3) = (int)(Fir_Dir_I(480,firBufferCounter,firFilter,firBuffer)*8388607.0f);
//			firBufferCounter = (firBufferCounter + 1) % 480;



//			output=sinf(2*PI*phase); //Freq Sweep
//			firBuffer[firBufferCounter]=output;
//			*(pDst+i+2) = (int)(output*8388607.0f);
//			*(pDst+i+3) = (int)(Fir_Dir_I(480,firBufferCounter,firFilter,firBuffer)*8388607.0f);
//			firBufferCounter = (firBufferCounter + 1) % 480;
//
//			phase = phase + sinFreq/SAMPLE_RATE;
//			if(phase>=1)
//				phase=phase-1;
//
//
//			if(capDelay<CAP_DELAY)
//				capDelay++;
//			else if(capCnt<SAMPLE_LENGTH)
//			{
//				xCap[capCnt] = (float)*(pSrc+i+2);
//				yCap[capCnt] = (float)*(pSrc+i+3);
//				capCnt++;
//			}
//			else if(capCnt==10800)
//			{
//				if(freqCnt<240)
//				{
//					capCnt=0;
//					capDelay=90000;
//					freqCnt++;
//					sinFreq=freqTable[freqCnt];
//				}
//				else
//				{
//					printf("%s\n","All End.");
//					capCnt=11111;
//				}
//			}
//			else if(capCnt>10000&capCnt<11000)
//			{
//				capCnt++;
//			}
//			else if(capCnt==10000)
//			{
//				crosscorrf(correlation,xCap,yCap,SAMPLE_LENGTH,CORR_LAG);
//				phs[freqCnt] = find_max(correlation, CORR_LAG) * sinFreq / SAMPLE_RATE * 2 * PI;
//				capCnt=10001;
//			}
//			else if(capCnt==SAMPLE_LENGTH)
//			{
//				xAmp = vector_norm(xCap,SAMPLE_LENGTH);
//				yAmp = vector_norm(yCap,SAMPLE_LENGTH);
//				mag[freqCnt] = yAmp/xAmp;
//				capCnt=10000;
//
//			}


//			delayBuffer[delayCnt]=output;
//			delayCnt = (delayCnt + 1) % 6;








			/* Channel 0 */
//			*(pDst+i) = *(pSrc+i); //Channel Thru, Comment to Mute
//			firBuffer[firBufferCounter] = (float)*(pSrc+i);//Channel FIR
//			float firOut=0;
//			for(int j=0; j<FIR_LENGTH;j++)
//			{
//				firOut = firOut + (firFilter[j] * firBuffer[(firBufferCounter - j + FIR_LENGTH) & (FIR_LENGTH - 1)]);
//			}
//			*(pDst+i) = (int) Fir_Dir_I(FIR_LENGTH,firBufferCounter,firFilter,firBuffer);
//			firBufferCounter = (firBufferCounter + 1) % FIR_LENGTH;
//			iirBuffer[0] = (float) *(pSrc+i);//Channel IIR
//			for(int j=0; j<IIR_STAGES; j++)
//			{
//				/* Direct Form I */
//				float temp;
//				temp = iirB[3 * j] * iirBuffer[3 * j] + iirB[3 * j + 1] * iirBuffer[3 * j + 1] + iirB[3 * j + 2] * iirBuffer[3 * j + 2];
//				iirBuffer[3 * (j + 1) + 2] = iirBuffer[3 * (j + 1) + 1];
//				iirBuffer[3 * (j + 1) + 1] = iirBuffer[3 * (j + 1)];
//				iirBuffer[3 * (j + 1) ] = temp - iirA[3 * j + 1] * iirBuffer[3 * (j + 1) + 1] - iirA[3 * j + 2] * iirBuffer[3 * (j + 1) + 2];
//			}
//			*(pDst+i) = (int)(iirBuffer[IIR_STAGES * 3]);
//			iirBuffer[2] = iirBuffer[1];
//			iirBuffer[1] = iirBuffer[0];
//			iirBuffer[iirCounter] = (float) *(pSrc+i);
//			for(int j=0; j<IIR_STAGES; j++)
//			{
//				/* Direct Form I */
//				float temp;
//				temp = iirB[3 * j] * iirBuffer[3 * j + iirCounter] + iirB[3 * j + 1] * iirBuffer[3 * j + (iirCounter - 1 + 3) % 3] + iirB[3 * j + 2] * iirBuffer[3 * j + (iirCounter - 2 + 3) % 3];
//				iirBuffer[3 * (j + 1) + iirCounter] = temp - iirA[3 * j + 1] * iirBuffer[3 * (j + 1) + (iirCounter - 1 + 3) % 3] - iirA[3 * j + 2] * iirBuffer[3 * (j + 1) + (iirCounter - 2 + 3) % 3];
//			}
//			*(pDst+i) = (int)(iirBuffer[IIR_STAGES * 3 + iirCounter]);
//			iirCounter = (iirCounter + 1) % 3;
//			iirBuffer[3 + iirCounter] = - iirB[0] * iirBuffer[iirCounter] - iirB[1] * iirBuffer[(iirCounter - 1 + 3) % 3] - iirB[2] * iirBuffer[(iirCounter - 2 + 3) % 3] - iirA[1] * iirBuffer[3 + (iirCounter - 1 + 3) % 3] - iirA[2] * iirBuffer[3 + (iirCounter - 2 + 3) % 3];
//			iirBuffer[6 + iirCounter] = - iirB[3] * iirBuffer[3 + iirCounter] - iirB[4] * iirBuffer[3 + (iirCounter - 1 + 3) % 3] - iirB[5] * iirBuffer[3 + (iirCounter - 2 + 3) % 3] - iirA[4] * iirBuffer[6 + (iirCounter - 1 + 3) % 3] - iirA[5] * iirBuffer[6 + (iirCounter - 2 + 3) % 3];
//			iirBuffer[9 + iirCounter] = - iirB[6] * iirBuffer[6 + iirCounter] - iirB[7] * iirBuffer[6 + (iirCounter - 1 + 3) % 3] - iirB[8] * iirBuffer[6 + (iirCounter - 2 + 3) % 3] - iirA[7] * iirBuffer[9 + (iirCounter - 1 + 3) % 3] - iirA[8] * iirBuffer[9 + (iirCounter - 2 + 3) % 3];
//			iirBuffer[12 + iirCounter] = - iirB[9] * iirBuffer[9 + iirCounter] - iirB[10] * iirBuffer[9 + (iirCounter - 1 + 3) % 3] - iirB[11] * iirBuffer[9 + (iirCounter - 2 + 3) % 3] - iirA[10] * iirBuffer[12 + (iirCounter - 1 + 3) % 3] - iirA[11] * iirBuffer[12 + (iirCounter - 2 + 3) % 3];
//			*(pDst+i) = (int) iirBuffer[12 + iirCounter];
//			iirBuffer[0][iirCounter] = (float) *(pSrc+i);
//			iirBuffer[1][iirCounter] = iirB[0] * iirBuffer[0][iirCounter] + iirB[1] * iirBuffer[0][(iirCounter - 1 + 3) % 3] + iirB[2] * iirBuffer[0][(iirCounter - 2 + 3) % 3] - iirA[1] * iirBuffer[1][(iirCounter - 1 + 3) % 3] - iirA[2] * iirBuffer[1][(iirCounter - 2 + 3) % 3];
//			iirBuffer[1][iirCounter] = iirB[0][0] * iirBuffer[0][iirCounter] + iirB[0][1] * iirBuffer[0][(iirCounter - 1) % 3] + iirB[0][2] * iirBuffer[0][(iirCounter - 2) % 3] - iirA[0][1] * iirBuffer[1][(iirCounter - 1) % 3] - iirA[0][2] * iirBuffer[1][(iirCounter - 2) % 3];
//			iirBuffer[2][iirCounter] = iirB[1][0] * iirBuffer[1][iirCounter] + iirB[1][1] * iirBuffer[1][(iirCounter - 1) % 3] + iirB[1][2] * iirBuffer[1][(iirCounter - 2) % 3] - iirA[1][1] * iirBuffer[2][(iirCounter - 1) % 3] - iirA[1][2] * iirBuffer[2][(iirCounter - 2) % 3];
//			iirBuffer[3][iirCounter] = iirB[2][0] * iirBuffer[2][iirCounter] + iirB[2][1] * iirBuffer[2][(iirCounter - 1) % 3] + iirB[2][2] * iirBuffer[2][(iirCounter - 2) % 3] - iirA[2][1] * iirBuffer[3][(iirCounter - 1) % 3] - iirA[2][2] * iirBuffer[3][(iirCounter - 2) % 3];
//			iirBuffer[4][iirCounter] = iirB[3][0] * iirBuffer[3][iirCounter] + iirB[3][1] * iirBuffer[3][(iirCounter - 1) % 3] + iirB[3][2] * iirBuffer[3][(iirCounter - 2) % 3] - iirA[3][1] * iirBuffer[4][(iirCounter - 1) % 3] - iirA[3][2] * iirBuffer[4][(iirCounter - 2) % 3];
//			*(pDst+i) = (int)(iirBuffer[1][iirCounter]);
//			iirBuffer[2 + iirCounter] = 0.0065 * iirBuffer[iirCounter] + 0.0065 * iirBuffer[(iirCounter - 1) % 2]  + 0.9870 * iirBuffer[2 + (iirCounter - 1) % 2];
//			*(pDst+i) = (int) iirBuffer[2 + iirCounter];
//			iirCounter = (iirCounter + 1) % 2;


			/* Channel 1 */
//			*(pDst+i+1) = *(pSrc+i+1); //Channel Thru, Comment to Mute
//			delaysample = delaysample%CHANNEL_DELAY_LENGTH; //Channel Delay
//			*(pDst+i+1) = delay[delaysample];
//			delay[delaysample] = *(pSrc+i+1);
//			delaysample++;


			/* Channel 2 */
//    		*(pDst+i+2) = *(pSrc+i+2); //Channel Thru, Comment to Mute

//			aaFilterBuffer[aaCnt] = (float)*(pSrc+i+2);//Downsample Toy
//			if (resampleCnt==0)
//			{
//				iirBuffer[0] = Fir_Dir_I(FIR_LENGTH,aaCnt,aaFilter,aaFilterBuffer);
//				for(int j=0; j<IIR_STAGES; j++)
//				{
//					/* Direct Form I */
//					float temp;
//					temp = iirB[3 * j] * iirBuffer[3 * j] + iirB[3 * j + 1] * iirBuffer[3 * j + 1] + iirB[3 * j + 2] * iirBuffer[3 * j + 2];
//					iirBuffer[3 * (j + 1) + 2] = iirBuffer[3 * (j + 1) + 1];
//					iirBuffer[3 * (j + 1) + 1] = iirBuffer[3 * (j + 1)];
//					iirBuffer[3 * (j + 1) ] = temp - iirA[3 * j + 1] * iirBuffer[3 * (j + 1) + 1] - iirA[3 * j + 2] * iirBuffer[3 * (j + 1) + 2];
//				}
//				iirBuffer[2] = iirBuffer[1];
//				iirBuffer[1] = iirBuffer[0];
//				usFilterBuffer[usCnt] = iirBuffer[IIR_STAGES * 3] * 6;
//				usFilterBuffer[usCnt] = Fir_Dir_I(FIR_LENGTH,aaCnt,aaFilter,aaFilterBuffer) * 6;
//				usCnt = (usCnt + 1) % (FIR_LENGTH / 6);
//			}
//			*(pDst+i+2) =  Upsampling_FIR(FIR_LENGTH,usCnt,resampleCnt,aaFilter,usFilterBuffer);
//			resampleCnt = (resampleCnt + 1) % 6;
//			aaCnt = (aaCnt + 1) % FIR_LENGTH;

//			aaFilterBuffer[aaCnt] = (float)*(pSrc+i+2);//Downsample Toy
//			if (resampleCnt==0)
//			{
//				inputBuffer[fftFilterCnt + FFT_LENGTH/2] = Fir_Dir_I(FIR_LENGTH,aaCnt,aaFilter,aaFilterBuffer);
//				usFilterBuffer[usCnt] = outputBuffer[fftFilterCnt] * RESAMPLE_RATIO;
//				if (fftFilterCnt==(FFT_LENGTH/2 - 1))
//				{
//					fft_filter(inputBuffer,outputBuffer);
//				}
//				fftFilterCnt = (fftFilterCnt + 1) % (FFT_LENGTH/2);
//				usCnt = (usCnt + 1) % (FIR_LENGTH / RESAMPLE_RATIO);
//			}
//			*(pDst+i+2) =  Upsampling_FIR(FIR_LENGTH,usCnt,resampleCnt,aaFilter,usFilterBuffer);
//			resampleCnt = (resampleCnt + 1) % RESAMPLE_RATIO;
//			aaCnt = (aaCnt + 1) % FIR_LENGTH;

//			blockInputBuffer[blockCnt + 1024] = (float)*(pSrc+i+2);//Block fft
//			*(pDst+i+2) = (int)(blockOutputBuffer[blockCnt]);
//			if (blockCnt==255)
//			{
//				block_fft_filter(blockInputBuffer,blockOutputBuffer);
//			}
//			blockCnt = (blockCnt + 1) % 256;



			/* Channel 3 */
//			*(pDst+i+3) = *(pSrc+i+3); //Channel Thru, Comment to Mute

		}
	    pDAC = NULL;
	    pADC = NULL;
#endif
#ifdef TDM4

		for(uint32_t i=0; i<(AUDIO_BUFFER_SIZE);i+=16)
		{
			pSrc =  (int8_t *)(int8_t *)pADC +i;

			for(uint32_t j=0; j<8; j++)
				{
					*pDst++ = *pSrc++;
				}
			pSrc = (int8_t *)(int8_t *)pADC + i;

			for(uint32_t K=8; K<16; K++)
				{
					*pDst++ = *pSrc++;
				}

		}
#endif
#ifdef TDM8

		for(uint32_t i=0; i<(AUDIO_BUFFER_SIZE);i+=32)
		{
			pSrc =  (int8_t *)(int8_t *)pADC +i;

			for(uint32_t j=0; j<8; j++)
				{
					*pDst++ = *pSrc++;
				}
			pSrc = (int8_t *)(int8_t *)pADC + i;

			for(uint32_t K=8; K<16; K++)
				{
					*pDst++ = *pSrc++;
				}
			pSrc = (int8_t *)(int8_t *)pADC + i;

			for(uint32_t x=16; x<24; x++)
				{
					*pDst++ = *pSrc++;
				}
			pSrc = (int8_t *)(int8_t *)pADC + i;
			for(uint32_t y=24; y<32; y++)
				{
					*pDst++ = *pSrc++;
				}

		}
#endif

#endif
}
	return (0u);
}

#pragma optimize_for_speed
void array_copy(float x[],float y[])
{
	for(int i=0;i<=256;i++)
	{
		y[i] = x[i];
	}
}

#pragma optimize_for_speed
void ps_update(float ps[], complex_float xf[])
{
	for(int i=0;i<=256;i++)
	{
		ps[i] = 0.85*ps[i] + xf[i].re*xf[i].re + xf[i].im*xf[i].im;
	}
}

#pragma optimize_for_speed
void noise_reduction(float np[], float ps[], complex_float xf[], complex_float yf[])
{
	for(int i = 0; i <= 256; i++)
	{
		Gain[i] = sqrtf(fabsf((ps[i] - np[i])/(ps[i] + 1e15)));
		if(Gain[i]>1.0f)
			Gain[i] = 1;

	}
	spectral_smooth(Gain,smoothWin,257,37,18);
	for(int i=0;i<=256;i++)
	{
		yf[i].re = xf[i].re * Gain[i];
		yf[i].im = xf[i].im * Gain[i];
	}

}

#pragma optimize_for_speed
void filter_upd(int filterLength,int filterCnt,float filter[],float zBuffer[],float error)
{
	int j = 0;
	int k = filterCnt;
	float err = error;
	float temp = err * 0.5 *1e-15;
	while(k>-1)
	{
		filter[j] = filter[j] - temp * zBuffer[k];
		j++;
		k--;
	}
	k = filterLength - 1;
	while(j<filterLength)
	{
		filter[j] = filter[j] - temp * zBuffer[k];
		j++;
		k--;
	}
}


#pragma optimize_for_speed
float vector_norm(float vector[],int length)
{
	float temp = 0;
	for(int i = 0;i<length;i++)
	{
		temp += vector[i]*vector[i];
	}
//	temp=sqrtf(temp);
	return temp;
}

#pragma optimize_for_speed
float Fir_Dir_I(int firLength,int firBufferCnt,float firFilterCoef[],float firFilterBuffer[])
{
	float firOut = 0;
	int j = 0;
	int k = firBufferCnt;
	while(k > -1)
	{
		firOut = firOut + (firFilterCoef[j] * firFilterBuffer[k]);
		j++;
		k--;
	}
	k = firLength - 1;
	while(j < firLength)
	{
		firOut = firOut + (firFilterCoef[j] * firFilterBuffer[k]);
		j++;
		k--;
	}
	return firOut;
}

#pragma optimize_for_speed
float Upsampling_FIR(int firLength,int firBufferCnt,int firCoeffCnt,float firFilterCoef[],float firFilterBuffer[])
{
	float firOut = 0;
	int j = firCoeffCnt;
	int k = firBufferCnt;
	while(k > -1)
	{
		firOut = firOut + (firFilterCoef[j] * firFilterBuffer[k]);
		j+=3;
		k--;
	}
	k = firLength / 3 - 1;
	while(j < firLength)
	{
		firOut = firOut + (firFilterCoef[j] * firFilterBuffer[k]);
		j+=3;
		k--;
	}
	return firOut;
}

#pragma optimize_for_speed
void spectral_smooth(float gain[],float win[],int fftLength,int winLength,int halfWin)
{
	int i;
	int j;
	int k;
	float temp = 0;
	for(i=0;i<halfWin;i++)
	{
		for(j=halfWin-i;j<winLength;j++)
		{
			k = j+i-halfWin;
			temp += gain[k]*win[j];
		}
		gain[i] = temp;
		temp = 0;
	}
	for(i=halfWin;i<(fftLength-halfWin);i++)
	{
		for(j=0;j<winLength;j++)
		{
			k = j+i-halfWin;
			temp += gain[k]*win[j];
		}
		gain[i] = temp;
		temp = 0;
	}
	for(i=fftLength-halfWin;i<fftLength;i++)
	{
		for(j=0;j<(halfWin+fftLength-i);j++)
		{
			k = j+i-halfWin;
			temp += gain[k]*win[j];
		}
		gain[i] = temp;
		temp = 0;
	}
}


//#pragma optimize_for_speed
//float cross_norm(float x[],float y[],int length)
//{
//	float temp = 0;
//	for(int i = 0;i<length;i++)
//	{
//		temp += x[i]*y[i];
//	}
//	return temp;
//}

//#pragma optimize_for_speed
//int find_max(const float x[], int length)
//{
//	int max;
//	float maxVal = 0;
//	for(int j=0;j<length;j++)
//	{
//		if(maxVal <= x[j])
//		{
//			maxVal = x[j];
//			max = j;
//		}
//	}
//	return max;
//}

//#pragma optimize_for_speed
//void NLMSSGD(float x[],float fit[],float error,int length,float stepSize)
//{
//	stepSize = stepSize / (vector_norm(x,length) + NLMS_DELTA);
//	for(int i=0;i < length;i++)
//	{
//		fit[i]+=stepSize * error * x[length - i - 1];
//	}
//}


//void main0()				// Cycle measurement, please replace "main0()" by "main()"
//{
//	uint32_t i =0u, i1,i2;
//	float temp = 0;
//	int *pSrc=NULL;
//	int *pDst=NULL;
//	void *pDAC;
//
//	CYCLES_INIT(stats);
//
//	for(i = 0; i < 256; i++)
//	{
//		CYCLES_START(stats);
//		inputBuffer[fftFilterCnt + FFT_LENGTH/2] = (float)*(pSrc+i);//fft
//		*(pDst+i) = (int)(outputBuffer[fftFilterCnt]);
//		if (fftFilterCnt==(FFT_LENGTH/2 - 1))
//		{
//			fft_filter(inputBuffer,outputBuffer);
//		}
//		fftFilterCnt = (fftFilterCnt + 1) % (FFT_LENGTH/2);
//		*(pDst+i) = *(pSrc+i); //Channel Thru, Comment to Mute
//		firBuffer[firBufferCounter] = (float)*(pSrc+i);//Channel FIR
//		*(pDst+i) = (int) Fir_Dir_I(FIR_LENGTH,firBufferCounter,firFilter,firBuffer);
//		firBufferCounter = (firBufferCounter + 1) % FIR_LENGTH;
//		CYCLES_STOP(stats);
//	}
//
//	printf("Cycles used by FIR filter\n");
//	CYCLES_PRINT(stats);
//}

//#pragma optimize_for_speed
//void fft_filter(float input[],float output[])
//{
//	rfft (input,r_temp,r_output,twiddle_table,1,FFT_LENGTH);
//	for (int i = 1; i < (FFT_LENGTH/2); i++)
//	{
//	    r_output[FFT_LENGTH - i] = conjf (r_output[i]);
//	}
//
//	for(int i = 0; i < FFT_LENGTH; i++)
//	{
//		r_output[i].re = fftFilterCoeff[i].re * r_output[i].re - fftFilterCoeff[i].im * r_output[i].im;
//		r_output[i].im = fftFilterCoeff[i].re * r_output[i].im + fftFilterCoeff[i].im * r_output[i].re;
//	}
//
//	ifft (r_output,i_temp,i_output,twiddle_table,1,FFT_LENGTH);
//	for (int i = 0; i < FFT_LENGTH/2; i++)
//	{
//		output[i] = i_output[i + (FFT_LENGTH/2)].re;
//	}
//	for (int i = 0; i < FFT_LENGTH/2; i++)
//	{
//		input[i] = input[i + (FFT_LENGTH/2)];
//	}
//}
//
//#pragma optimize_for_speed
//void block_fft_filter(float input[],float output[])
//{
//	int i = 0;
//	for (i = 0; i < 256; i++)
//	{
//		output[i] = middleBuffer[i];
//	}
//
//	for (i = 0; i < 256; i++)
//	{
//		middleBuffer[i] = 0;
//	}
//
//	for (i = 0; i < 512; i++)
//	{
//		i_input[i] = input[i];
//	}
//	rfft (i_input,r_temp,r_output,twiddle_table,1,FFT_LENGTH);
//	for (i = 1; i < (FFT_LENGTH/2); i++)
//	{
//	    r_output[FFT_LENGTH - i] = conjf (r_output[i]);
//	}
//	for(i = 0; i < FFT_LENGTH; i++)
//	{
//		r_output[i].re = B4[i].re * r_output[i].re - B4[i].im * r_output[i].im;
//		r_output[i].im = B4[i].re * r_output[i].im + B4[i].im * r_output[i].re;
//	}
//	ifft (r_output,i_temp,i_output,twiddle_table,1,FFT_LENGTH);
//	for (i = 0; i < 256; i++)
//	{
//		middleBuffer[i] += i_output[i + 256].re;
//	}
//
//	for (i = 0; i < 512; i++)
//	{
//		i_input[i] = input[i + 256];
//	}
//	rfft (i_input,r_temp,r_output,twiddle_table,1,FFT_LENGTH);
//	for (i = 1; i < (FFT_LENGTH/2); i++)
//	{
//	    r_output[FFT_LENGTH - i] = conjf (r_output[i]);
//	}
//	for(i = 0; i < FFT_LENGTH; i++)
//	{
//		r_output[i].re = B3[i].re * r_output[i].re - B3[i].im * r_output[i].im;
//		r_output[i].im = B3[i].re * r_output[i].im + B3[i].im * r_output[i].re;
//	}
//	ifft (r_output,i_temp,i_output,twiddle_table,1,FFT_LENGTH);
//	for (i = 0; i < 256; i++)
//	{
//		middleBuffer[i] += i_output[i + 256].re;
//	}
//
//	for (i = 0; i < 512; i++)
//	{
//		i_input[i] = input[i + 512];
//	}
//	rfft (i_input,r_temp,r_output,twiddle_table,1,FFT_LENGTH);
//	for (i = 1; i < (FFT_LENGTH/2); i++)
//	{
//	    r_output[FFT_LENGTH - i] = conjf (r_output[i]);
//	}
//	for(i = 0; i < FFT_LENGTH; i++)
//	{
//		r_output[i].re = B2[i].re * r_output[i].re - B2[i].im * r_output[i].im;
//		r_output[i].im = B2[i].re * r_output[i].im + B2[i].im * r_output[i].re;
//	}
//	ifft (r_output,i_temp,i_output,twiddle_table,1,FFT_LENGTH);
//	for (i = 0; i < 256; i++)
//	{
//		middleBuffer[i] += i_output[i + 256].re;
//	}
//
//	for (i = 0; i < 512; i++)
//	{
//		i_input[i] = input[i + 768];
//	}
//	rfft (i_input,r_temp,r_output,twiddle_table,1,FFT_LENGTH);
//	for (i = 1; i < (FFT_LENGTH/2); i++)
//	{
//	    r_output[FFT_LENGTH - i] = conjf (r_output[i]);
//	}
//	for(i = 0; i < FFT_LENGTH; i++)
//	{
//		r_output[i].re = B1[i].re * r_output[i].re - B1[i].im * r_output[i].im;
//		r_output[i].im = B1[i].re * r_output[i].im + B1[i].im * r_output[i].re;
//	}
//	ifft (r_output,i_temp,i_output,twiddle_table,1,FFT_LENGTH);
//	for (i = 0; i < 256; i++)
//	{
//		middleBuffer[i] += i_output[i + 256].re;
//	}
//
//	for (i = 0; i < 1024; i++)
//	{
//		input[i] = input[i + 256];
//	}
//}
//
//





/*
 * Generate a sine wave and fill a buffer.
 *
 * Parameters
 *  Frequency	- Sine wave frequency in Hz
 *  SampleRate 	- Sample rate of ADC/DAC in Hz
 *  pBuffer		- Pointer to the buffer where the sine wave will be placed
 *  samples		- Number of samples generated
 *  numchannels - Number of audio channels
 *
 * Returns
 *  0 if success, other values for error
 *
 */

uint32_t Generate_SineWave(uint32_t Frequency, uint32_t SampleRate, void * pBuffer, uint32_t samples, uint32_t numchannels)
{

	uint32_t i;
	uint32_t mod;
	int32_t *p;
	p = pBuffer;

	/* sine wave frequency must be a multiple of the sample rate,
	 * otherwise waveform will not be continuous
	 */
	mod = SampleRate % Frequency;
	if(mod != 0)
	{
		return (1u);
	}

	/* number of samples in the sine wave must to a multiple of the number of
	 * samples in 1 period, otherwise waveform will not be continuous
	 */
	mod = samples % (SAMPLES_PER_PERIOD);
	if(mod != 0)
	{
		return (1u);
	}

	/* create out sine wave */
	for( i = 0u; i < samples ; i++ )
	{
#ifdef TDM_MODE
		*(p + numchannels * i) = ((int32_t)(AMPLITUDE * sin((2.0 * PI * Frequency * (((float)(i)) / ((float)SampleRate))))) << 8);
#else
		*(p + numchannels * i) = (int32_t)(AMPLITUDE * sin((2.0 * PI * Frequency * (((float)(i)) / ((float)SampleRate)))));
#endif
	}

	return (0u);
}

/*
 * Generates a correlation table from a specified audio buffer.
 *
 * Parameters
 *  pBuffer				- Pointer to the buffer of the waveform to be correlated
 *  samples				- Number of samples to correlate
 *  stride				- The stride width that separates each sample
 *  pCoorelationTable	- Pointer to the generated correlation table
 *
 * Returns
 *  0 if success, other values for error
 *
 */

uint32_t Correlation(int32_t * pBuffer, uint32_t samples, uint32_t stride, int32_t * pCoorelationTable)
{
	uint32_t i, j;
	uint32_t mod;
	int32_t *p;
	int32_t *p1;
	int32_t *pCorr;
	int32_t Sum = 0u;
	int32_t Total = 0u;

	/* number of samples to correlate must be a multiple
	 * of the number of samples in 1 period
	 */
	mod = samples % (SAMPLES_PER_PERIOD);
	if(mod != 0)
	{
		return (1u);
	}

	/* number of sample to correlate must at least twice
	 * the number of samples in 1 period so the
	 * sliding window correlation can be performed
	 */
	if(samples < (SAMPLES_PER_PERIOD) * 2u)
	{
		return (1u);
	}

	pCorr = pCoorelationTable;
	/* reset correlation table */
	for( i = 0u; i < samples ; i++ )
	{
		*pCorr++ = 0u;
	}

	pCorr = pCoorelationTable;

	for( i = 0u; i < samples ; i++ )
	{
		/* pointer to reference correlation starting point */
		p     = (int32_t *)(pBuffer);
		/* pointer to correlation sliding window starting point,
		 * x samples ahead of reference correlation
		 * */
		p1    = (int32_t *)(pBuffer + (i * stride));
		for( j = 0u; j < samples; j++ )
		{
			/* scale then multiply to avoid overflow */
			Sum = (*(p+ (j * stride)) >> CORRELATION_SCALING_FACTOR1) * (*(p1+(j * stride)) >> CORRELATION_SCALING_FACTOR1);
			/* accumulate */
			Total += (Sum >> CORRELATION_SCALING_FACTOR2);
		}
		/* store multiplication/accumulation in the correlation table */
		*pCorr++ = Total;
		Total = 0u;
		Sum = 0u;
	}
	return (0u);

}

/*
 * Analyze a correlation table to determine if its contents represent a specified sine wave frequency
 *
 * Parameters
 *  ref_freq			- reference frequency in Hz
 *  samples				- Number of samples to correlate
 *  pCoorelationTable	- Pointer to the generated correlation table
 *
 * Returns
 *  0 if success, other values for error
 *
 */

/* Analyzes a correlation table to determine if its contents represents the specified sine wave frequency  */
uint32_t Check_Correlation(uint32_t ref_freq, uint32_t samples, int32_t * pCoorelationTable)
{
	uint32_t result = 0u;
	uint32_t i;
	uint32_t i_peak;
	int32_t *pCorr;

	int32_t largest_corr;
	int32_t next_largest_corr;

	pCorr = pCoorelationTable;

	/* The maximum correlation is when sine wave is in phase represented
	 * by the first correlated sample. The next maximum occurs one correlation sample
	 * later.
	 */
	largest_corr      = *pCorr;
	next_largest_corr = *(pCorr +1);

	if(next_largest_corr >= largest_corr)
	{
		/* there has been an error */
		return 1u;
	}

	/* Search for 2nd correlation maximum.
	 * This should occur when the  sine wave is in phase again
	 * ie. 1 period of samples later
	 */
	next_largest_corr = *(pCorr + 1u);
	for( i = 1u; i < samples - 2u ; i++ )
	{
		if(*(pCorr + i) > next_largest_corr)
		{
			next_largest_corr = *(pCorr + i);
			i_peak = i;
		}
	}
	if(i_peak != SAMPLE_RATE/ref_freq)
	{
		/* the 2nd correlation maximum should be 16 samples away from 1st
		 * correlation maximum, assuming a sample rate of 48kHz and input of 3kHz
		 */
		result = 1u;
	}
#if 0
	/* check the maximum and minimum allowable tolerance */
	if((next_largest_corr > (CORRELATION_NORM_PEAK + CORRELATION_NORM_TOLERANCE)) ||
	   (next_largest_corr < (CORRELATION_NORM_PEAK - CORRELATION_NORM_TOLERANCE)))
	{
		result = 1u;
	}
#endif

	/* check the maximum and minimum allowable tolerance */
	if(next_largest_corr < 0x100000u)
	{
		result = 1u;
	}

	return (result);
}


/*
 * ADC Callback.
 *
 * Parameters
 *  None
 *
 * Returns
 *  None
 *
 */
void AdcCallback(void *pCBParam, uint32_t nEvent, void *pArg)
{
    switch(nEvent)
    {
        case ADI_SPORT_EVENT_RX_BUFFER_PROCESSED:
        	/* Update callback count */
        	AdcCount++;
        	/* store pointer to the processed buffer that caused the callback */
        	pGetADC = pArg;
        	break;
        default:
        	bEventError = true;
            break;
    }
}

/*
 * DAC Callback.
 *
 * Parameters
 *  None
 *
 * Returns
 *  None
 *
 */
void DacCallback(void *pCBParam, uint32_t nEvent, void *pArg)
{
    switch(nEvent)
    {
        case ADI_SPORT_EVENT_TX_BUFFER_PROCESSED:
        	/* Update callback count */
        	DacCount++;
        	/* store pointer to the processed buffer that caused the callback */
        	pGetDAC = pArg;
        	break;
        default:
        	bEventError = true;
            break;
    }
}

/*****/
