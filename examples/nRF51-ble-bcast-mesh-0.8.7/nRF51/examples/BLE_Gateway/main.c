/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#include "rbc_mesh.h"
#ifdef MESH_DFU
#include "dfu_app.h"
#endif
#ifdef BLINKY
#include "blinky.h"
#endif

#include "nrf_adv_conn.h"
#include "led_config.h"
#include "mesh_aci.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"

#if NORDIC_SDK_VERSION >= 11
#include "nrf_nvic.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "app_timer.h"
#include "bsp.h"
#include "nrf_drv_config.h"
#if DEBUG_LOG_RTT
#include "SEGGER_RTT.h"
#endif
#include "nrf_delay.h"
#include "app_pwm.h"
#include "nrf_error.h"
#include "nrf_adc.h"
#include "nrf_wdt.h"
#include "nrf_drv_wdt.h"
#include "app_util_platform.h"
//#include "ble_bas.h"
#include "ble_dis.h"
//#include "ble_srv_common.h"
#include "ble_temperature.h"
#include "ble_light.h"

#define DEVICE_NAME                  "LIGHT_SWITCH" /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME            "TEMCOCONTROLS"     	 /**< Manufacturer. Will be passed to Device Information Service. */
#define DEVICE_HARDWARE_VERSION		 "v3B"					 /* Device hardware version */
#define DEVICE_FIRMWARE_VERSION		 __DATE__				 /* Device firmware version */
#define DEVICE_SOFTWARE_VERSION		 __TIME__
#define DEVICE_SERIAL_NUMBER		 "0123-4567-89AB"		 /* device serial number */

#define MESH_ACCESS_ADDR        	(RBC_MESH_ACCESS_ADDRESS_BLE_ADV)   /**< Access address for the mesh to operate on. */
#define MESH_INTERVAL_MIN_MS    	(100)                               /**< Mesh minimum advertisement interval in milliseconds. */
#define MESH_CHANNEL            	(38)                                /**< BLE channel to operate on. Single channel only. */

#define APP_TIMER_PRESCALER        	(0)                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS       	(12 + BSP_APP_TIMERS_NUMBER)      	/**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE    	(4)                                 /**< Size of timer operation queues. */

#define HEARTBEAT_INTERVAL      APP_TIMER_TICKS(130, APP_TIMER_PRESCALER) 	/**< led1 heartbeat interval (ticks). */
#define RELAY_INTERVAL      	APP_TIMER_TICKS(90, APP_TIMER_PRESCALER) 	/**< RELAY interval (ticks). */
#define COMMUNICATE_INTERVAL    APP_TIMER_TICKS(50, APP_TIMER_PRESCALER) 	/**< communicate led interval (ticks). */
#define MOTION_SOUND_INTERVAL   APP_TIMER_TICKS(3760, APP_TIMER_PRESCALER) 	/**< SOUND&MOTION led interval (ticks). */
/* a man breath rates 16-20 per minute, T = 3s - 3.75s, step length 40, so interval 75-93 */
#define PWM_UPDATE_INTERVAL     APP_TIMER_TICKS(85, APP_TIMER_PRESCALER) 	
#define PIR_MES_INTERVAL    	APP_TIMER_TICKS(180, APP_TIMER_PRESCALER) 	/**< sound measure interval (ticks). */
#define SOUND_MES_INTERVAL    	APP_TIMER_TICKS(12, APP_TIMER_PRESCALER) 	/**< sound measure interval (ticks). */
#define LIGHT_MES_INTERVAL    	APP_TIMER_TICKS(540, APP_TIMER_PRESCALER) 	/**< light sonsor measure interval (ticks). */
#define TEMP_MES_INTERVAL    	APP_TIMER_TICKS(620, APP_TIMER_PRESCALER) 	/**< temperature measure interval (ticks). */
#define WATCHDOG_INTERVAL    	APP_TIMER_TICKS(505, APP_TIMER_PRESCALER) 	/**< watchdog interval (ticks). */
#define KEY_PERIOD_HANG_SENSOR	APP_TIMER_TICKS(3009, APP_TIMER_PRESCALER)  /**< detect key hang motion and sound interval (ticks). */
#define TRIGGER_INTERVAL		APP_TIMER_TICKS(300000, APP_TIMER_PRESCALER)  /**< TRIGGER THEN DELAY OFF(ticks). */

/**@brief Timer status. */
typedef enum
{
    APP_TIMER_STOP,                 /**< The timer no initial or stoped. */
	APP_TIMER_START                 /**< The timer already started. */
} app_timer_status_t;

static app_timer_id_t           m_heartbeat_timer_id;         /**< heartbeat timer. */
static app_timer_status_t		s_heartbeat_timer;			  /**< status of heartbeat timer. */	
#ifdef RELAY_LATCH
static app_timer_id_t           m_relay_timer_id;             /**< relay execute timer. */
static app_timer_status_t		s_relay_timer;				  /**< status of relay execute timer. */
#endif
static app_timer_id_t           m_communicate_timer_id;       /**< led communicate timer. */
static app_timer_status_t		s_communicate_timer;		  /**< status of led communicate timer. */
static app_timer_id_t           m_motion_sound_timer_id;      /**< motion and sound event timer. */
static app_timer_status_t		s_motion_sound_timer;		  /**< status of motion and sound event timer. */
#ifdef BREATH_LED
static app_timer_id_t           m_pwm_update_timer_id;        /**< PWM update timer. */
static app_timer_status_t		s_pwm_update_timer;			  /**< status of PWM update timer. */
#endif
static app_timer_id_t           m_pir_mes_timer_id;           /**< PIR measure timer. */
static app_timer_status_t		s_pir_mes_timer;			  /**< status of PIR measure timer. */
static app_timer_id_t           m_sound_mes_timer_id;         /**< sound measure timer. */
static app_timer_status_t		s_sound_mes_timer;			  /**< status of sound measure timer. */
static app_timer_id_t           m_light_mes_timer_id;         /**< light sensor measure timer. */
static app_timer_status_t		s_light_mes_timer; 			  /**< status of light sensor measure timer. */
static app_timer_id_t           m_temp_mes_timer_id;          /**< temperature measure timer. */
static app_timer_status_t		s_temp_mes_timer;			  /**< status of temperature measure timer. */
static app_timer_id_t           m_watchdog_timer_id;          /**< watchdog timer. */
static app_timer_status_t		s_watchdog_timer; 			  /**< status of watchdog timer. */
static app_timer_id_t           m_hang_on_timer_id;           /**< hang on sound & pir timer. */
static app_timer_status_t		s_hang_on_timer;			  /**< status of hang on sound & pir timer. */
static app_timer_id_t           m_trigger_timer_id; 		  /**< darkness occpuy sensor trigger timer. */	
static app_timer_status_t		s_trigger_timer;  			  /**< status of trigger timer. */

static ble_temp_t               m_temp;     /**< Structure used to identify the battery service. */
static ble_light_t              m_light;
static volatile bool ready_flag;            /* A flag indicating PWM status. */
volatile int32_t adc_sample[4];
volatile int32_t PIR_Buffer[2];
volatile int32_t sound_sample = 0;
volatile uint32_t light_sample = 0;
volatile int32_t temp_sample = 0;
volatile int16_t temp_centigrade;
volatile int16_t temp_fahrenheit; 
volatile int16_t calibration_temperature = 0;

static  uint8_t  pir_triggle_mode = 0;
led_event_t			 led_event;
pwm_enevt_t			 pwm_event;
uint8_t 			 relay_status = 0;
nrf_drv_wdt_channel_id m_channel_id;		//wdt
void update_led_event(led_event_e led_event_type);
void relay_on(void);
void relay_off(void);

uint16_t const ntc_table_10K[20] =	
{
	25,	39,	61,	83,	102, 113, 112, 101,	85,
	67,	51,	38,	28,	21,	15,	11,	8, 6, 5, 19
};

uint16_t const lightsensor_table_tps851[27] =	
{
	  1,  2,  2,  4,  6,  7,   9, 12, 14, 16,
	 18, 20, 37, 63, 80, 94, 117,142,154,171,
	196,313,455,597,711,853, 1024
//	0x0001, 0x0002, 0x0002, 0x0004, 0x0006, 0x0007, 0x0009, 0x000c, 0x000e, 0x0010,
//	0x0012,	0x0014,	0x0025,	0x003f,	0x0050,	0x005e,	0x0075,	0x008e, 0x009a, 0x00ab, 
//	0x00c4, 0x0139, 0x01c7, 0x0255, 0x02c7, 0x0355, 0x0400
};

APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.


#if NORDIC_SDK_VERSION >= 11
nrf_nvic_state_t nrf_nvic_state = {0};
static nrf_clock_lf_cfg_t m_clock_cfg = 
{
    .source = NRF_CLOCK_LF_SRC_XTAL,    
    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_75_PPM
};

#define MESH_CLOCK_SOURCE       (m_clock_cfg)    /**< Clock source used by the Softdevice. For calibrating timeslot time. */
#else
#define MESH_CLOCK_SOURCE       (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)    /**< Clock source used by the Softdevice. For calibrating timeslot time. */
#endif

#ifdef NRF51
#define EXAMPLE_DFU_BANK_ADDR   (0x26000)
#endif
#ifdef NRF52
#define EXAMPLE_DFU_BANK_ADDR   (0x40000)
#endif

static void trigger_handler(void * p_context)
{
	if(pir_triggle_mode)
	{
		
		pir_triggle_mode = 0;
		relay_status = 0;
		relay_off();	
		nrf_gpio_pin_clear(LED_1);
		nrf_gpio_pin_clear(LED_3);			
	}
	
}


void shut_pir_sound_timer(void)
{
	uint32_t err_code;	
	err_code = app_timer_stop(m_pir_mes_timer_id);
	APP_ERROR_CHECK(err_code);	
	err_code = app_timer_stop(m_sound_mes_timer_id);
	APP_ERROR_CHECK(err_code);	
}
	
void open_pir_sound_timer(void)
{
    uint32_t err_code;
	err_code = app_timer_start(m_pir_mes_timer_id, PIR_MES_INTERVAL, NULL);				
    APP_ERROR_CHECK(err_code);	
    err_code = app_timer_start(m_sound_mes_timer_id, SOUND_MES_INTERVAL, NULL);			
    APP_ERROR_CHECK(err_code);	
}
#ifdef BREATH_LED
void update_pwm_value(uint32_t value)	
{	
	while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
	while (app_pwm_channel_duty_set(&PWM1, 1, value) == NRF_ERROR_BUSY);	
}
#endif
int16_t look_up_table(int16_t adc)
{                                        
    int16_t val, work_var;
    int16_t index = 19;
       
    work_var = ntc_table_10K[index]; 
    if(work_var > adc)
    {
        val = (index - 4) * 100; // >= max range, so use the last point of array
        return ((int16_t)val);                                                         
    }                         
    do                        
    {
        index--;
        work_var += ntc_table_10K[index];    //check which range match current value:L14 + (L13 - L14) + (L12-L13) + ... + [L (x - 1) - Lx] + [Lx - L (x + 1) ] = L (x + 1)
        if(work_var > adc)    //can check relative document in : Z:\Designs\Temperature\Curves\ThermistorCurves_VoltageCalcs.xls
        {
            val = (work_var - adc) * 100;    //get the difference value between current value and nearest default point
            val /= ntc_table_10K[index];
            if(index >= 4)
            {
                val += ((index - 4) * 100);    //get nearest temperature point value
                val &= 0x7fff;
            }
            else
            {
                val += index * 100;
                val = 400 - val;
                val |= 0x8000;
            }                     
            return ((int16_t)val);        
        }                 
    } while(index);                                                                      

//	val = index * 100; 
	val = 400 | 0x8000;
    return ((int16_t)val);  
}

int16_t update_temperature(int16_t adc, _Bool c)
{
	int16_t deg_temp = (int16_t)(look_up_table(adc));
	if(deg_temp & 0x8000) // minus temperature
		deg_temp = -(signed int)(deg_temp & 0x7fff);
	
	temp_centigrade = deg_temp + calibration_temperature;
	temp_fahrenheit = temp_centigrade * 9 / 5 + 320;

//	if((output_auto_manual & 0x01) == 0x01)
//		temp_centigrade = output_manual_value_temp;
	if(c)
		return temp_centigrade;
	else 
		return temp_fahrenheit;		
}

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
    LEDS_OFF(LEDS_MASK);
 
#if 1	
	for(uint8_t i= 0; i <= 20; i++)
	{
		nrf_delay_ms(50);
		nrf_gpio_pin_toggle(BSP_LED_0);
	}	
#endif	
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}
/**
 * @brief ADC  handler.
 */
void get_channel_adc(void)
{
		adc_sample[0] = nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_2);	
		adc_sample[1] = nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_3);	
		adc_sample[2] = nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_4);
		adc_sample[3] = nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_5);	

//#ifdef DEBUG_LOG_RTT									
//		SEGGER_RTT_printf(0, "adc:%4d,%4d,%4d,%4d \r\n",(uint16_t)adc_sample[0],(uint16_t)adc_sample[1],(uint16_t)adc_sample[2],(uint16_t)adc_sample[3]);		
//#endif		
}	
	
/**
 * @brief ADC initialization.
 */
void adc_config(void)
{
    const nrf_adc_config_t nrf_adc_config = NRF_ADC_CONFIG_DEFAULT;

// 	Initialize and configure ADC
    nrf_adc_configure( (nrf_adc_config_t *)&nrf_adc_config);
//    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_4);
//    nrf_adc_int_enable(ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos);
//    NVIC_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_HIGH);
//    NVIC_EnableIRQ(ADC_IRQn);
}
#ifdef BREATH_LED
void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}
#endif
void leds_on_for_while(void)
{
	uint8_t i;
	
	for( i= 0; i <= 20; i++)
	{
		nrf_delay_ms(50);
		nrf_gpio_pin_toggle(BSP_LED_1);
	}
		
	nrf_gpio_pin_clear(BSP_LED_0);
	nrf_gpio_pin_clear(BSP_LED_1);
	nrf_gpio_pin_clear(BSP_LED_2);
	nrf_gpio_pin_clear(BSP_LED_3);
}

#ifdef RELAY_LATCH
void relay_on(void)
{
	nrf_gpio_pin_clear(RELAY_OFF);
	nrf_gpio_pin_set(RELAY_ON);
}
void relay_off(void)
{
	nrf_gpio_pin_clear(RELAY_ON);
	nrf_gpio_pin_set(RELAY_OFF);
}
#else
void relay_on(void)
{
	nrf_gpio_pin_set(13);
}
void relay_off(void)
{
	nrf_gpio_pin_clear(13);
}
#endif

void relay_idle(void)
{
	nrf_gpio_pin_clear(RELAY_ON);
	nrf_gpio_pin_clear(RELAY_OFF);
}

void led_communicate_blink(led_event_e led_event_type)
{
	//UNUSED_PARAMETER(led_event_type);
	update_led_event(led_event_type);
	
	nrf_gpio_pin_set(BSP_LED_1);
	uint32_t err_code = app_timer_start(m_communicate_timer_id, COMMUNICATE_INTERVAL, NULL);
	APP_ERROR_CHECK(err_code);	
}
void motion_sound_event_set(led_event_e led_event_type)
{
	update_led_event(led_event_type);
	nrf_gpio_pin_set(BSP_LED_1);	
	
	uint32_t err_code = app_timer_start(m_motion_sound_timer_id, MOTION_SOUND_INTERVAL, NULL);
	APP_ERROR_CHECK(err_code);	

#ifdef BREATH_LED	
	memset(&pwm_event,0,sizeof(pwm_event));
	err_code = app_timer_start(m_pwm_update_timer_id, PWM_UPDATE_INTERVAL, &pwm_event);
	APP_ERROR_CHECK(err_code);	
#endif	
}




void update_led_event(led_event_e led_event_type)
{
	led_event.status = true ;	//led bright on first.
	
	if(led_event_type == SOUND_EVENT) 
	{
		led_event.event_type = SOUND_EVENT; 
		led_event.off_time = SOUND_EVENT_OFF;
		led_event.on_time = SOUND_EVENT_ON;		
	}else if (led_event_type == MOTION_EVENT)
	{
		led_event.event_type = MOTION_EVENT; 
		led_event.off_time = MOTION_EVENT_OFF;
		led_event.on_time = MOTION_EVENT_ON;					
	}else if(led_event_type == HEARTBEAT_EVENT)  
	{
		led_event.event_type = HEARTBEAT_EVENT;
		led_event.off_time = HEARTBEAT_EVENT_OFF;
		led_event.on_time = HEARTBEAT_EVENT_ON;					
	}else if(led_event_type == COMMUNICATE_EVENT)  
	{
		led_event.event_type = COMMUNICATE_EVENT;
		led_event.off_time = HEARTBEAT_EVENT_OFF;
		led_event.on_time = HEARTBEAT_EVENT_ON;					
	}
	
}

/**@brief Function for handling the led blink timer timeout.
 *
 * @details This function will be called each time the led blink timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void led_heartbeat_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);		
		
	if((led_event.event_type == SOUND_EVENT) || (led_event.event_type == MOTION_EVENT))
	{

	}else if(led_event.event_type == COMMUNICATE_EVENT)
	{
		
	}else if(led_event.event_type == HEARTBEAT_EVENT)
	{
		if(led_event.status)
		{
			if(!led_event.on_time--)
			{					
				nrf_gpio_pin_clear(BSP_LED_1);
				led_event.status = false;
				led_event.off_time = (uint16_t)HEARTBEAT_EVENT_OFF; 
				led_event.on_time = (uint16_t)HEARTBEAT_EVENT_ON; 
			}			
		}else
		{
			if(!led_event.off_time--)
			{			
				nrf_gpio_pin_set(BSP_LED_1);
				led_event.status = true;
				led_event.on_time = (uint16_t)HEARTBEAT_EVENT_ON;
				led_event.off_time = (uint16_t)HEARTBEAT_EVENT_OFF; 		
			}			
		}		
	}else if(led_event.event_type == FIRMWAREUPDATE_EVENT)
	{
		
	}else
	{
		
	}
		
}
/**@brief Function for handling the watchdog timer timeout.
 *
 * @details This function will be called each time the watchdog timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void watch_dog_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);	
	nrf_drv_wdt_channel_feed(m_channel_id);	//feed the dog
}


/**@brief Function for handling the relay execute timer timeout.
 *
 * @details This function will be called each time relay execute timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
#ifdef RELAY_LATCH
static void relay_execute_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);		
    relay_idle();		
}
#endif
/**@brief Function for handling the led stay timer timeout.
 *
 * @details This function will be called each led stay execute timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void led_off_handler(void * p_context)
{
    uint32_t err_code;
	
	UNUSED_PARAMETER(p_context);		
    nrf_gpio_pin_clear(BSP_LED_1);	
	//update_led_event(HEARTBEAT_EVENT);
	
	led_event.status = false;
	led_event.event_type = HEARTBEAT_EVENT;
	led_event.off_time = HEARTBEAT_EVENT_OFF - 40;			//queick start blink ASAP.
	led_event.on_time = HEARTBEAT_EVENT_ON;		
	
    err_code = app_timer_start(m_pir_mes_timer_id, PIR_MES_INTERVAL, NULL);				
    APP_ERROR_CHECK(err_code);	
    err_code = app_timer_start(m_sound_mes_timer_id, SOUND_MES_INTERVAL, NULL);			
    APP_ERROR_CHECK(err_code);	
}
/**@brief Function for handling the led communicate timer timeout.
 *
 * @details This function will be called each led communicate execute timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void blink_off_handler(void * p_context)
{	
	UNUSED_PARAMETER(p_context);		
    nrf_gpio_pin_clear(BSP_LED_1);	
	update_led_event(HEARTBEAT_EVENT);			
}

/**@brief Function for handling the pir&sound hang on timer timeout.
 *
 * @details This function will be called each led pir&sound hang on execute timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void hang_on_handler(void * p_context)
{	
	
	UNUSED_PARAMETER(p_context);		

	open_pir_sound_timer();
	
}
#ifdef BREATH_LED
/**@brief Function for handling the pwm update timer timeout.
 *
 * @details This function will be called each pem update timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void pwm_update_handler(void * p_context)
{
    uint32_t		err_code;
	uint32_t		value ;	
	pwm_enevt_t	*	p_pe;     
	
	p_pe = (pwm_enevt_t*) p_context;
	p_pe->loop++;		

	if(p_pe->loop <= 40)
	{										
		//if(p_pe->tendency)
		if(relay_status)
		{									
			value = (p_pe->loop < 20) ? (100 - p_pe->loop * 5) : (p_pe->loop * 5 - 100);	/*¡ü ¡ý*/	
		}else
		{			
			value = (p_pe->loop < 20) ? (p_pe->loop * 5) : (100 - (p_pe->loop - 20) * 5);	/*¡ý ¡ü*/
		}
					
		ready_flag = false;
		/* Set the duty cycle - keep trying until PWM is ready... */
		while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
		;
		while (app_pwm_channel_duty_set(&PWM1, 1, value) == NRF_ERROR_BUSY);
		;
		/* ... or wait for callback. */
		//while(!ready_flag);
		//APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, value));

//#ifdef DEBUG_LOG_RTT
//		SEGGER_RTT_printf(0, "pwm_update %d,%d.\r\n",(uint8_t)p_pe->loop,(uint16_t)value);
//#endif					
		
	}else
	{
		err_code = app_timer_stop(m_pwm_update_timer_id);
		APP_ERROR_CHECK(err_code);			
	}			
}
#endif
/**@brief Function for handling the PIR update timer timeout.
 *
 * @details This function will be called each PIR update timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void pir_event_handler(void * p_context)
{
	uint32_t err_code;
	
	UNUSED_PARAMETER(p_context);	
	//PIR_Buffer[0] = PIR_Buffer[1];
	
	PIR_Buffer[1] = nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_4);	
	
	if(PIR_Buffer[1] < 200) 
	{
		update_led_event(MOTION_EVENT);
		motion_sound_event_set(MOTION_EVENT);
				
#ifdef DEBUG_LOG_RTT
		SEGGER_RTT_printf(0, "MOTION_EVENT %2d\r\n",(uint16_t)PIR_Buffer[1]);															
#endif	
		if((light_sample < 10)&&(!pir_triggle_mode)) /* NO Enter */
		{
			pir_triggle_mode = 1;
						
			relay_status = 1;
			relay_on();			
			
			nrf_gpio_pin_set(LED_1);
			nrf_gpio_pin_set(LED_3);			
			
			err_code = app_timer_start(m_trigger_timer_id, TRIGGER_INTERVAL, NULL);		/* TRIGGER OFF */
			APP_ERROR_CHECK(err_code);			

#ifdef DEBUG_LOG_RTT
		SEGGER_RTT_printf(0, "enter trigger mode.\r\n");															
#endif				
			
		}		
		shut_pir_sound_timer();		
	}
		
}
/**@brief Function for handling the sound event timer timeout.
 *
 * @details This function will be called each sound event timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sound_event_handler(void * p_context)
{
	uint32_t err_code;
	
	UNUSED_PARAMETER(p_context);	
	
	sound_sample = nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_2);	
	
	if(sound_sample > 400) 
	{
		update_led_event(SOUND_EVENT);
		motion_sound_event_set(SOUND_EVENT);
		
#ifdef DEBUG_LOG_RTT
		SEGGER_RTT_printf(0, "SOUND_EVENT %2d\r\n",(uint16_t)sound_sample);															
#endif	

		err_code = app_timer_stop(m_pir_mes_timer_id);
		APP_ERROR_CHECK(err_code);	
		err_code = app_timer_stop(m_sound_mes_timer_id);
		APP_ERROR_CHECK(err_code);		
	}
		
}

/**@brief Function for handling the light event timer timeout.
 *
 * @details This function will be called each light event timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void light_event_handler(void * p_context)
{
	//uint32_t l1;
	uint32_t err_code;
	UNUSED_PARAMETER(p_context);	
	
	light_sample = (uint32_t) nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_3);		
	light_sample *= 527;
	light_sample /= 2000;		/* k = 1891 */

	err_code = ble_light_sensor_level_update(&m_light,(uint16_t)light_sample);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }	
	
#ifdef DEBUG_LOG_RTT	
	SEGGER_RTT_printf(0, "LUX:%2d\r\n",(uint16_t)light_sample);															
#endif		
			
}
/**@brief Function for handling the temperature event timer timeout.
 *
 * @details This function will be called each temperature event timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void temperature_event_handler(void * p_context)
{
	uint32_t err_code;
	
	UNUSED_PARAMETER(p_context);				
	temp_sample = update_temperature((int16_t)nrf_adc_convert_single(NRF_ADC_CONFIG_INPUT_5), true);
	
	err_code = ble_temp_Temperature_level_update(&m_temp, (uint16_t)temp_sample);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }	
//#ifdef DEBUG_LOG_RTT
//	SEGGER_RTT_printf(0, "temp:%2d\r\n",(int16_t)temp_sample);															
//#endif		
			
}

/**
* @brief General error handler.
*/
static void error_loop(void)
{
    led_config(0, 1);
    led_config(1, 1);		// led1
    led_config(2, 1);
    led_config(3, 1);
    
    __disable_irq(); /* Prevent the mesh from continuing operation. */
    while (true)
    {
        __WFE(); /* sleep */
    }
}
/**
* @brief Softdevice crash handler, never returns
*
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
#ifdef DEBUG_LOG_RTT
		SEGGER_RTT_printf(0, "[sd_assert_handler],pc:%2d,line_num:%6d,file_name:%s.\r\n",(uint16_t)pc,(uint16_t)line_num,p_file_name);
#endif    
	error_loop();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
*   Never returns.
*
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
#ifdef DEBUG_LOG_RTT
	SEGGER_RTT_printf(0, "[app_error_handler],error_code: %d, line_num: %d, file_name:%s\r\n",(uint16_t)error_code,
																	(uint16_t)line_num, p_file_name);
#endif  
#if 0	
	while(true)
	{
		for(uint8_t i=0;i<=20;i++)
		{
			nrf_delay_ms(50);
			nrf_gpio_pin_toggle(BSP_LED_2);			
		}
	};
#endif	
	//error_loop();
}

/** @brief Hardware fault handler. */
void HardFault_Handler(void)
{
#ifdef DEBUG_LOG_RTT
		SEGGER_RTT_printf(0, "HardFault_Handler.\r\n  ");
#endif   

	error_loop();
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
#ifdef DEBUG_LOG_RTT
	SEGGER_RTT_printf(0, "[app_error_fault],ID: %d,PC: %d,info: %d.\r\n",id,pc,info);
#endif
#if 0	
	while(true)
	{
		for(uint8_t i=0;i<=50;i++)
		{
			nrf_delay_ms(300);
			nrf_gpio_pin_toggle(BSP_LED_2);			
		}
	};
#endif	
    //error_loop();
}
/**
* @brief Softdevice event handler
*/
void sd_ble_evt_handler(ble_evt_t* p_ble_evt)
{
    rbc_mesh_ble_evt_handler(p_ble_evt);
    nrf_adv_conn_evt_handler(p_ble_evt);
}

/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Sets LEDs according to data
*
* @param[in] evt RBC event propagated from framework
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* p_evt)
{   
    led_communicate_blink(COMMUNICATE_EVENT);
	switch (p_evt->type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            if (p_evt->params.rx.value_handle > 1)
                break;

            //led_config(p_evt->params.rx.value_handle, p_evt->params.rx.p_data[0]);
			
            break;
        case RBC_MESH_EVENT_TYPE_TX:
            break;

        case RBC_MESH_EVENT_TYPE_INITIALIZED:
            /* init BLE gateway softdevice application: */
            nrf_adv_conn_init();
            break;
#ifdef MESH_DFU
        case RBC_MESH_EVENT_TYPE_DFU_BANK_AVAILABLE:
            dfu_bank_flash(p_evt->params.dfu.bank.dfu_type);
            break;

        case RBC_MESH_EVENT_TYPE_DFU_NEW_FW_AVAILABLE:
            dfu_request(p_evt->params.dfu.new_fw.dfu_type,
                &p_evt->params.dfu.new_fw.new_fwid,
                (uint32_t*) EXAMPLE_DFU_BANK_ADDR);
            break;

        case RBC_MESH_EVENT_TYPE_DFU_RELAY_REQ:
            dfu_relay(p_evt->params.dfu.relay_req.dfu_type,
                &p_evt->params.dfu.relay_req.fwid);
            break;

        case RBC_MESH_EVENT_TYPE_DFU_START:
        case RBC_MESH_EVENT_TYPE_DFU_END:
            break;
        case RBC_MESH_EVENT_TYPE_DFU_SOURCE_REQ:
            break;
#endif
        default:
            break;
    }
}


/**
* @brief Initialize GPIO pins, for LEDs and debugging
*/
void gpio_init(void)
{
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    for (uint32_t i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_pin_set(LED_START + i);
    }	
	nrf_gpio_pin_set(BSP_LED_1);	// LOW LEVEL LED1 ON
	
#if defined(BOARD_LIGHTSWITCH)			
	nrf_gpio_cfg_output(RELAY_OFF);
	nrf_gpio_pin_clear(RELAY_OFF);
	nrf_gpio_cfg_output(RELAY_ON);
	nrf_gpio_pin_clear(RELAY_ON);	
	nrf_gpio_cfg_output(RELAY2_ON);	
	nrf_gpio_pin_clear(RELAY2_ON);
	nrf_gpio_cfg_output(RELAY2_OFF);	
	nrf_gpio_pin_clear(RELAY2_OFF);	
    nrf_gpio_cfg_output(IR_SEND);
	nrf_gpio_pin_clear(IR_SEND);
	nrf_gpio_cfg_input(IR_REC,NRF_GPIO_PIN_PULLUP);		
#endif

#if defined(BOARD_PCA10028) || defined(BOARD_PCA10040)
    #ifdef BUTTONS
        nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_3, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_4, NRF_GPIO_PIN_PULLUP);
    #endif
#endif
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_KEY_0:
		
        case BSP_EVENT_KEY_1:	

		pir_triggle_mode = 0;				/* if key press exit trigger mode */
		app_timer_stop(m_trigger_timer_id);
		
		if(relay_status)
		{			
			relay_status = 0;

#ifdef BREATH_LED			
			update_pwm_value(0x00);
#else			
	
			
			nrf_gpio_pin_set(LED_1);
			nrf_gpio_pin_set(LED_3);				
			
#endif			
			shut_pir_sound_timer();		
			err_code = app_timer_start(m_hang_on_timer_id, KEY_PERIOD_HANG_SENSOR, NULL);			
			APP_ERROR_CHECK(err_code);	

#ifdef RELAY_LATCH	
			relay_off();
			err_code = app_timer_start(m_relay_timer_id, RELAY_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);	
#else
			relay_off();
#endif			

#ifdef DEBUG_LOG_RTT
			SEGGER_RTT_printf(0, "SWITCH,OFF.\r\n");	
#endif			
		}else
		{
			
			relay_status = 1;
#ifdef BREATH_LED			
			update_pwm_value(100);	
#else			

			nrf_gpio_pin_clear(LED_1);
			nrf_gpio_pin_clear(LED_3);

			
#endif			
#ifdef RELAY_LATCH
			relay_on();
			err_code = app_timer_start(m_relay_timer_id, RELAY_INTERVAL, NULL);
			APP_ERROR_CHECK(err_code);
#else
			relay_on();
#endif
			shut_pir_sound_timer();	
			err_code = app_timer_start(m_hang_on_timer_id, KEY_PERIOD_HANG_SENSOR, NULL);			
			APP_ERROR_CHECK(err_code);
			
#ifdef DEBUG_LOG_RTT
			SEGGER_RTT_printf(0, "SWITCH,ON.\r\n");
#endif
		}		
        break;
        case BSP_EVENT_KEY_2:
				
        break;
        case BSP_EVENT_KEY_3:
				
        break;
        default:
            break;
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

	/* The timer library enables the application to create multiple timer instances based on the RTC1 peripheral. 
		Checking for time-outs and invoking the user time-out handlers is performed in the RTC1 interrupt handler. 
		List handling is done using a software interrupt (SWI0). Both interrupt handlers are running in APP_LOW priority level. */	
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
		
    // Create timers.
    err_code = app_timer_create(&m_heartbeat_timer_id, APP_TIMER_MODE_REPEATED, led_heartbeat_handler);                                                                
    APP_ERROR_CHECK(err_code);
	// Create relay execute timer.
#ifdef RELAY_LATCH
    err_code = app_timer_create(&m_relay_timer_id, APP_TIMER_MODE_SINGLE_SHOT, relay_execute_handler);                                                                
    APP_ERROR_CHECK(err_code);	
#endif
	
    err_code = app_timer_create(&m_communicate_timer_id, APP_TIMER_MODE_SINGLE_SHOT, blink_off_handler);                                                                
    APP_ERROR_CHECK(err_code);	
	
#ifdef BREATH_LED
    err_code = app_timer_create(&m_pwm_update_timer_id, APP_TIMER_MODE_REPEATED, pwm_update_handler);                                                                
    APP_ERROR_CHECK(err_code);
#endif	
    err_code = app_timer_create(&m_motion_sound_timer_id, APP_TIMER_MODE_SINGLE_SHOT, led_off_handler);                                                                
    APP_ERROR_CHECK(err_code);	
	
    err_code = app_timer_create(&m_pir_mes_timer_id, APP_TIMER_MODE_REPEATED, pir_event_handler);                                                                
    APP_ERROR_CHECK(err_code);	
	
    err_code = app_timer_create(&m_sound_mes_timer_id, APP_TIMER_MODE_REPEATED, sound_event_handler);                                                                
    APP_ERROR_CHECK(err_code);		
		
    err_code = app_timer_create(&m_light_mes_timer_id, APP_TIMER_MODE_REPEATED, light_event_handler);                                                                
    APP_ERROR_CHECK(err_code);		
	
    err_code = app_timer_create(&m_temp_mes_timer_id, APP_TIMER_MODE_REPEATED, temperature_event_handler);                                                                
    APP_ERROR_CHECK(err_code);		
	
    err_code = app_timer_create(&m_watchdog_timer_id, APP_TIMER_MODE_REPEATED, watch_dog_handler);                                                                
    APP_ERROR_CHECK(err_code);	

    err_code = app_timer_create(&m_hang_on_timer_id, APP_TIMER_MODE_SINGLE_SHOT, hang_on_handler);                                                                
    APP_ERROR_CHECK(err_code);	
	
    err_code = app_timer_create(&m_trigger_timer_id, APP_TIMER_MODE_SINGLE_SHOT, trigger_handler);                                                                
    APP_ERROR_CHECK(err_code);	
}
/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_sound_mes_timer_id, SOUND_MES_INTERVAL, NULL);	/* sound detect timer */
    APP_ERROR_CHECK(err_code);		
	
    err_code = app_timer_start(m_watchdog_timer_id, WATCHDOG_INTERVAL, NULL);	/* watchdog feed */
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_heartbeat_timer_id, HEARTBEAT_INTERVAL, NULL);	/* hearybeat timer */
    APP_ERROR_CHECK(err_code);
		
    err_code = app_timer_start(m_pir_mes_timer_id, PIR_MES_INTERVAL, NULL);		/* PIR detect timer */
    APP_ERROR_CHECK(err_code);		
	
    err_code = app_timer_start(m_light_mes_timer_id, LIGHT_MES_INTERVAL, NULL); /* light sensor measurement */
    APP_ERROR_CHECK(err_code);	
	
    err_code = app_timer_start(m_temp_mes_timer_id, TEMP_MES_INTERVAL, NULL);	/* temperature measurement */
    APP_ERROR_CHECK(err_code);


}
/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(void)
{
    uint32_t err_code = bsp_init(/*BSP_INIT_LED |*/ BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

}
#ifdef BREATH_LED
void bsp_pwm_init(void)
{
    ret_code_t err_code;
    
    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(5000L, BSP_LED_0, BSP_LED_2);
    
    /* Switch the polarity of the second channel. */
	pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;	//APP_PWM_POLARITY_ACTIVE_HIGH
    	
    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,NULL);	//pwm_ready_callback
    APP_ERROR_CHECK(err_code);
		
    app_pwm_enable(&PWM1);	
	
	while (app_pwm_channel_duty_set(&PWM1, 0, 100) == NRF_ERROR_BUSY);	
	while (app_pwm_channel_duty_set(&PWM1, 1, 100) == NRF_ERROR_BUSY);
}
#endif
void bsp_wdt_init(void)
{
	uint32_t err_code = NRF_SUCCESS;
	
    //Configure WDT.
	nrf_drv_wdt_config_t config;	
	config.behaviour = NRF_WDT_BEHAVIOUR_PAUSE_SLEEP_HALT;
	config.interrupt_priority = APP_IRQ_PRIORITY_HIGH;
	config.reload_value = 2000;	
	
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();	
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_temp_init_t temp_init;
	ble_light_init_t light_init;
	ble_dis_init_t dis_init;


    /* Initialize Light Sensor Service.*/
    memset(&light_init, 0, sizeof(light_init));

    /* Here the sec level for the light sensor Service can be changed/increased. */
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&light_init.light_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&light_init.light_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&light_init.light_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&light_init.light_level_report_read_perm);

    light_init.evt_handler          = NULL;
    light_init.support_notification = true;
    light_init.p_report_ref         = NULL;
    light_init.initial_light_level   = 100;

    err_code = ble_light_init(&m_light, &light_init);
    APP_ERROR_CHECK(err_code);		
	
	
    /* Initialize Temperature Service.*/
    memset(&temp_init, 0, sizeof(temp_init));

    /* Here the sec level for the Temperature Service can be changed/increased. */
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.temperature_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.temperature_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&temp_init.temperature_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.temperature_level_report_read_perm);

    temp_init.evt_handler          = NULL;
    temp_init.support_notification = true;
    temp_init.p_report_ref         = NULL;
    temp_init.initial_temp_level   = 100;

    err_code = ble_temp_init(&m_temp, &temp_init);
    APP_ERROR_CHECK(err_code);	
	
	
    // Initialize Device Information Service. 
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
	ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)DEVICE_NAME);
	ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)DEVICE_HARDWARE_VERSION);
	ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)DEVICE_FIRMWARE_VERSION);
	ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, (char *)DEVICE_SOFTWARE_VERSION);
	ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char *)DEVICE_SERIAL_NUMBER);
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);	
	
}

/** @brief main function */
int main(void)
{		
	
    /* init leds and pins */
    gpio_init();
	/* power on led light on for a while */
	leds_on_for_while();
    /* Initialize timer module.*/
    timers_init();
	/* Initialize button and leds.*/
	buttons_leds_init();
#ifdef BREATH_LED	
	/* Initialize breath led pwm */
	bsp_pwm_init();
#endif	
	/* Initialize adc config.*/
	adc_config();
	/* Initialize wdt modle */
	bsp_wdt_init();
	
#ifdef DEBUG_LOG_RTT	// RTT debug interface.
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
	
	SEGGER_RTT_printf(0, "BUILD DATE:[%s %s].\r\n", __DATE__,__TIME__);
	SEGGER_RTT_printf(0, "RESET_REASON:%2x\r\n",(uint16_t)NRF_POWER->RESETREAS);
			
	NRF_POWER->RESETREAS = 0xffffffff;
#endif	
	
    /* Enable Softdevice (including sd_ble before framework */
#ifdef NRF52
    SOFTDEVICE_HANDLER_INIT(&MESH_CLOCK_SOURCE,NULL); 
#else
    SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SOURCE,NULL); 
#endif    

    softdevice_ble_evt_handler_set(sd_ble_evt_handler); 
	/* app-defined event handler, as we need to send it to the nrf_adv_conn module and the rbc_mesh */
    softdevice_sys_evt_handler_set(rbc_mesh_sd_evt_handler);

#ifdef RBC_MESH_SERIAL
    mesh_aci_init();
#endif   
       
    rbc_mesh_init_params_t init_params;

    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SOURCE;
    init_params.tx_power = RBC_MESH_TXPOWER_0dBm ;
    
    uint32_t error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);

	nrf_drv_wdt_channel_feed(m_channel_id);
    /* request values for both LEDs on the mesh */
    for (uint32_t i = 0; i < 2; ++i)
    {
        error_code = rbc_mesh_value_enable(i);
        APP_ERROR_CHECK(error_code);
    }
    /* init BLE gateway softdevice application: */
    nrf_adv_conn_init();
    
#ifdef RBC_MESH_SERIAL
    APP_ERROR_CHECK(mesh_aci_start());
#endif
    
	services_init();
	
#ifdef BLINKY   
    led_init ();
    rtc_1_init ();
    start_blink_interval_s(1);    
#endif  
		
    app_button_enable();
//	app_pwm_enable(&PWM1);
	application_timers_start();	
//	motion_sound_event_set(HEARTBEAT_EVENT);
//	rbc_mesh_stop();
    rbc_mesh_event_t evt;
    while (true)
    {
#ifdef BUTTONS
        for (uint32_t pin = BUTTON_START; pin <= BUTTON_STOP; ++pin)
        {
            if(nrf_gpio_pin_read(pin) == 0)
            {
                while(nrf_gpio_pin_read(pin) == 0);
                uint8_t mesh_data[1];
                uint32_t led_status = !!((pin - BUTTON_START) & 0x01); /* even buttons are OFF, odd buttons are ON */
                uint32_t led_offset = !!((pin - BUTTON_START) & 0x02); /* two buttons per led */

                mesh_data[0] = led_status;
                if (rbc_mesh_value_set(led_offset, mesh_data, 1) == NRF_SUCCESS)
                {
                    led_config(led_offset, led_status);
                }
            }
        }
#endif       
        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {   
            rbc_mesh_event_handler(&evt);
			nrf_drv_wdt_channel_feed(m_channel_id);
            rbc_mesh_event_release(&evt);			
        }						
		sd_app_evt_wait();
		
		nrf_drv_wdt_channel_feed(m_channel_id);
				
    }
}

