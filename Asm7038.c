/*
 * Asm7038.c
 *
 *  Created on: Jun 26, 2023
 *      Author: saber
 *
 *      Main poriton of the functions derived from https://github.com/Chasbrot/AS7038RB/blob/main/AS7038RB.c
 */
#include "main.h"
#include "stm32wbxx_hal_i2c.h"
#include "Asm7038.h"
#include "stdbool.h"

#define POX_SAMPLES 2
uint16_t pox_samples[POX_SAMPLES];
uint8_t buf[12];
extern I2C_HandleTypeDef hi2c1;
extern uint8_t Value;
extern uint8_t ret;

//################   Define the functions    ###############################

uint8_t as7038_write_random(uint8_t reg, uint8_t data){
	uint8_t tx_data[2] = {reg, data};
	uint8_t temp = HAL_I2C_Master_Transmit(&hi2c1, AS7038_I2C_ADDRESS, tx_data, 2, HAL_MAX_DELAY);
	HAL_Delay(50);
	return temp;
}

uint8_t as7038_write_single(uint8_t reg, uint8_t data){
	uint8_t tx_data[1] = {reg};
	return HAL_I2C_Master_Transmit(&hi2c1, AS7038_I2C_ADDRESS, tx_data, 1, HAL_MAX_DELAY);
}

uint8_t as7038_read_single(uint8_t reg, uint8_t* data){
	return HAL_I2C_Master_Receive(&hi2c1, AS7038_I2C_ADDRESS, data, 1, HAL_MAX_DELAY);
}

uint8_t as7038_read_random(uint8_t reg, uint8_t *data)
{
	uint8_t err = as7038_write_single(reg, 0);
	err += as7038_read_single(reg, data);
	HAL_Delay(50);
    return err;
}

void as7038_set_bitmask(uint8_t reg, uint8_t mask)
{
    uint8_t buf;
    as7038_read_random(reg, &buf);
    buf |= mask;
    as7038_write_random(reg, buf);
}


void as7038_led_setMode(as7038_led_t led, as7038_led_mode_t mode)
{
    if (mode == AS7038_LED_ALWAYS_OFF)
    {
        switch (led)
        {
        case AS7038_LED_1:
            as7038_clear_bitmask(AS7038_LED12_MODE, 0x07);
            break;
        case AS7038_LED_2:
            as7038_clear_bitmask(AS7038_LED12_MODE, 0x70);
            break;
        case AS7038_LED_3:
            as7038_clear_bitmask(AS7038_LED34_MODE, 0x07);
            break;
        case AS7038_LED_4:
            as7038_clear_bitmask(AS7038_LED34_MODE, 0x70);
            break;
        }
    }
    else
    {
        switch (led)
        {
        case AS7038_LED_1:
//            as7038_set_bitmask(AS7038_LED12_MODE, 0x99);
            as7038_set_bitmask(AS7038_LED12_MODE, mode);

            break;
        case AS7038_LED_2:
            as7038_set_bitmask(AS7038_LED12_MODE, (mode << 4));
            break;
        case AS7038_LED_3:
//            as7038_set_bitmask(AS7038_LED34_MODE, 0x99);
            as7038_set_bitmask(AS7038_LED34_MODE, mode);

            break;
        case AS7038_LED_4:
            as7038_set_bitmask(AS7038_LED34_MODE, (mode << 4));
            break;
        }
    }
}



uint8_t as7038_read_fifo(uint16_t *array)
{
    uint8_t anz, fifol, fifoh;
    as7038_read_random(AS7038_FIFOLEVEL, &anz);
    for (int i = 0; i < anz && i < AS7038_FIFO_BLOCK; i++)
    {
        as7038_read_random(AS7038_FIFOL, &fifol);
        as7038_read_random(AS7038_FIFOH, &fifoh);
        uint16_t buff = (fifoh << 8);
        buff |= fifol;
        array[i] = buff;
    }
    return anz;
}

uint8_t as7038_led_current(as7038_led_t led, as7038_led_current_t current)
{
    // Write first 2 Bytes
    uint8_t err = as7038_write_random(AS7038_LED1_CURRH + (led * 2), (current >> 2));
    // Write last byte
    err += as7038_write_random(AS7038_LED1_CURRL + (led * 2), (current << 6));
    return err;
}


void as7038_prefilter_setGain(as7038_prefilter_gain_t gain)
{
    uint8_t buf;
    as7038_read_random(AS7038_OFE_CFGA, &buf);
    if (gain == AS7038_PREFILTER_GAIN_1)
    {
        buf &= ~(AS7038_PREFILTER_AA_60kHz);
    }
    else
    {
        buf |= gain;
    }
    as7038_write_random(AS7038_OFE_CFGA, buf);
}


void as7038_prefilter_setAAFreq(as7038_prefilter_aa_freq_t freq)
{
    uint8_t buf;
    as7038_read_random(AS7038_OFE_CFGA, &buf);
    if (freq == AS7038_PREFILTER_AA_10kHz)
    {
        buf &= ~(freq);
    }
    else
    {
        buf |= freq;
    }
    as7038_write_random(AS7038_OFE_CFGA, buf);
}


void as7038_led_activate(as7038_led_t led)
{
    uint8_t cfg;
    as7038_read_random(AS7038_LED_CFG, &cfg);
    cfg |= 1UL << led;
    as7038_write_random(AS7038_LED_CFG, cfg);
}

void as7038_led_deactivate(as7038_led_t led)
{
    uint8_t cfg;
    as7038_read_random(AS7038_LED_CFG, &cfg);
    cfg &= ~(1UL << led);
    as7038_write_random(AS7038_LED_CFG, cfg);
}


void as7038_clear_bitmask(uint8_t reg, uint8_t mask)
{
    uint8_t buf;
    as7038_read_random(reg, &buf);

    buf &= ~(mask);

    as7038_write_random(reg, buf);
}


void as7038_pd_config(as7038_pd_config_t pd, bool en)
{
    if (en)
    {
        as7038_set_bitmask(AS7038_PD_CFG, pd);
    }
    else
    {
        as7038_clear_bitmask(AS7038_PD_CFG, pd);
    }
}


void as7038_tia_setGain(as7038_tia_gain_t gain)
{
    as7038_write_random(AS7038_PD_AMPCFG, gain | 0x80); //ENABLE
    as7038_write_random(AS7038_PD_AMPRCCFG, gain >> 8);
}


void as7038_seq_samplingPoint(uint8_t start){
    as7038_write_random(AS7038_SEQ_ADC, start);
}

void as7038_seq_windowITG(uint8_t start, uint8_t e){
    as7038_write_random(AS7038_SEQ_ITG_STA, start);
    as7038_write_random(AS7038_SEQ_ITG_STO, e);
}
/*
* Start or Stop cannot be zero
*/
void as7038_seq_windowLED(uint8_t start, uint8_t e){
    as7038_write_random(AS7038_SEQ_LED_STA, start);
    as7038_write_random(AS7038_SEQ_LED_STO, e);
}

void as7038_seq_clockDivider(uint8_t d){
    as7038_write_random(AS7038_SEQ_DIV, d);
}

void as7038_seq_cycleDuration(uint8_t duration){
    as7038_write_random(AS7038_SEQ_PER, duration);
}


void as7038_seq_enable(bool en){
    if(en){
        as7038_set_bitmask(AS7038_MAN_SEQ_CFG, 0x01);
    }else{
        as7038_clear_bitmask(AS7038_MAN_SEQ_CFG, 0x01);
    }
}

void as7038_adc_setChannel(as7038_adc_channel_t channel)
{
    uint8_t maskl, maskh;
    as7038_read_random(AS7038_ADC_CHANNEL_MASK_L, &maskl);
    as7038_read_random(AS7038_ADC_CHANNEL_MASK_H, &maskh);
    maskl |= channel;
    maskh |= channel >> 8;
    as7038_write_random(AS7038_ADC_CHANNEL_MASK_H, maskh);
    as7038_write_random(AS7038_ADC_CHANNEL_MASK_L, maskl);
}

void as7038_adc_setClock(as7038_adc_clock_t c)
{
    as7038_write_random(AS7038_ADC_CFGB, c);
}


void as7038_adc_setSettlingTime(as7038_adc_settling_time_t t)
{
    as7038_write_random(AS7038_ADC_CFGC, t);
}

void as7038_adc_enable()
{
    uint8_t t;
    as7038_read_random(AS7038_ADC_CFGB, &t);
    t |= 0x01;
    as7038_write_random(AS7038_ADC_CFGB, t);
}


void as7038_prefilter_enableStage(as7038_prefilter_config_t stage)
{
    as7038_set_bitmask(AS7038_OFE_CFGC, stage);
}

void as7038_prefilter_disableStage(as7038_prefilter_config_t stage)
{
    as7038_clear_bitmask(AS7038_OFE_CFGC, stage);
}

uint8_t as7038_pox_start()
{
    // Enable PD1
	//////   AS7038_PD_CFG  (0x1A); 7-> NA   6-> Boost  5-> pd4  4-> pd3  3 ->  pd2   2-> pd1   1 -> pd-i1   0 -> pd-i0

//    as7038_pd_config(AS7038_PD_1, TRUE);
//    as7038_pd_config(AS7038_PD_2, TRUE);
//    as7038_pd_config(AS7038_PD_3, TRUE);
//    as7038_pd_config(AS7038_PD_4, TRUE);

    ret += as7038_write_random(AS7038_PD_CFG, 0x3c);

//    // Enable LED
//    as7038_led_setMode(AS7038_LED_1, AS7038_LED_ALWAYS_ON);
//    as7038_led_setMode(AS7038_LED_2, AS7038_LED_ALWAYS_ON);
//    as7038_led_setMode(AS7038_LED_3, AS7038_LED_ALWAYS_ON);
//    as7038_led_setMode(AS7038_LED_4, AS7038_LED_ALWAYS_ON);


	//////////    AS7038_LED12_MODE  (0x2c);   7 -> man-sw_led2    6:4  led2_mode    3->  man_sw_led1   2:0->  led1_mode

    ret += as7038_write_random(AS7038_LED34_MODE, 0xAA);
    ret += as7038_write_random(AS7038_LED12_MODE, 0xAA);

//    as7038_led_enabled(AS7038_LED_1, true);
//    as7038_led_enabled(AS7038_LED_2, true);
//    as7038_led_enabled(AS7038_LED_3, true);
//    as7038_led_enabled(AS7038_LED_4, true);

    /////////  AS7038_LED_CFG   (0x10);    7 -> Sigref-en   6 -> sigref-ecg-vol    5:4 -> sigref-ofe-vol   3 -> led4-en  2 -> led3-en  1 -> led2-en   0 -> led1-en
    ret += as7038_write_random(AS7038_LED_CFG, 0x8f);

    // Enable pd_amp

    as7038_tia_setGain(AS7038_TIA_GAIN_3VuA_14);

    // Configure Prefilter
    ///////////////////   0x52    OFE Configuration
    ///////////////////		1 -> not used 6-> aa_by 5-> 200hz HP_filter_by 4-> prefilter_gain_by 3-> bypass_en 2-> aa_en 1-> 200hz_HP_en 0-> gain_en

//    as7038_prefilter_enableStage(AS7038_PREFILTER_AA_BYPASS);
//    as7038_prefilter_enableStage(AS7038_PREFILTER_GAIN_STAGE);
//    as7038_prefilter_enableStage(AS7038_PREFILTER_HP_FILTER);
    ret += as7038_write_random(AS7038_OFE_CFGC, 0x07);


    /////////////////  Set the gain of prefilter
//    as7038_prefilter_setGain(AS7038_PREFILTER_GAIN_4);
    ret += as7038_write_random(AS7038_OFE_CFGA, 0xe6);


    // Configure ADC
//    as7038_adc_setSettlingTime(AS7038_ADC_SETTLING_TIME_4);
    ret += as7038_write_random(AS7038_ADC_CFGC, 0x01);

//    as7038_adc_setChannel(AS7038_ADC_CHANNEL_TIA);
    //////////////////////    Connecting the gain stage output after second demodulator to the ADC
    ret += as7038_write_random(AS7038_ADC_CHANNEL_MASK_L, 0x01);
    ret += as7038_write_random(AS7038_ADC_CHANNEL_MASK_H, 0x00);

//    as7038_adc_setClock(0x39);
    as7038_adc_setClock(AS7038_ADC_CLOCK_1000kHz);
    as7038_adc_enable();

    // Configure Sequencer

//##################   0x42
    as7038_seq_samplingPoint(64);
    as7038_seq_windowLED(1, 128);
    as7038_seq_clockDivider(16);
    as7038_seq_cycleDuration(255);

//    as7038_seq_enable(true);

    as7038_write_random(AS7038_MAN_SEQ_CFG, 0x01);

    // Setup Interrupt vector
    //as7038_irq_setVector(AS7038_IRQ_VECTOR_FIFOOVERFLOW,true);

    return ret;
}


uint8_t as7038_seq_active(bool en){
    if(en){
        as7038_set_bitmask(AS7038_SEQ_START, 0x01);
    }else{
        as7038_clear_bitmask(AS7038_SEQ_START, 0x01);
    }
    return 0;
}

uint8_t Blinking_LEDs(){

    // Enable LED1 manual mode and always on when seq is on

	//////////    AS7038_LED12_MODE  (0x2c);   7 -> man-sw_led2    6:4  led2_mode    3->  man_sw_led1   2:0->  led1_mode
	as7038_write_random(AS7038_LED12_MODE, 0x09);
    as7038_write_random(AS7038_LED34_MODE, 0x09);

    // Enable seq and manual mode
    /////    AS7038_MAN_SEQ_CFG   (0x2E);  7-> man_mode   6-> man_sw_sd_mult  5-> an_sw_sd_pol  4-> man_sw_itg    3:1  -> diode_ctrl   0-> seq_en
    as7038_write_random(AS7038_MAN_SEQ_CFG, 0xf0);

//  ########## make sure just enable Seg-ref and not the ECG seg-ref. otherwise it wont be able to handle all LED
    as7038_write_random(AS7038_LED_CFG, 0xc5);

    // Enable LED1, blinky blink
    for (int i = 0; i < 3; i++)
    {
		HAL_Delay(500);
		as7038_write_random(AS7038_LED_CFG, 0x8f);

//		as7038_led_activate(AS7038_LED_1);
//		as7038_led_activate(AS7038_LED_2);
//		as7038_led_activate(AS7038_LED_3);
//		as7038_led_activate(AS7038_LED_4);
		HAL_Delay(500);
//		as7038_write_random(AS7038_LED_CFG, 0xc0);
//
////		as7038_led_deactivate(AS7038_LED_1);
////		as7038_led_deactivate(AS7038_LED_2);
////		as7038_led_deactivate(AS7038_LED_3);
////		as7038_led_deactivate(AS7038_LED_4);
    }

    // Disable seq and manual mode
    as7038_write_random(AS7038_MAN_SEQ_CFG, 0x00);
    as7038_write_random(AS7038_LED12_MODE, 0x00);

    return 0;
}


uint8_t as7038_activate_LEDs_LDO(ldo_gpio_pin pin){
	uint8_t cfg;
    uint8_t err = as7038_read_random(AS7038_GPIO_O, &cfg);
    cfg |= 1UL << pin;
    err += as7038_write_random(AS7038_GPIO_O, cfg);
    return err;
}


void as7038_led_enabled(as7038_led_t led, bool en)
{
    if (en)
    {
        as7038_set_bitmask(AS7038_LED_CFG, ((uint8_t)led ));
//        as7038_set_bitmask(AS7038_LED_CFG, 0xff);
    }
    else
    {
        as7038_clear_bitmask(AS7038_LED_CFG, ((uint8_t)led ));
    }
}


uint8_t as7038_config_gpio_output(ldo_gpio_pin pin){
	uint8_t cfg;
    uint8_t err = as7038_read_random(AS7038_GPIO_E, &cfg);
    cfg |= 1UL << pin;
    err += as7038_write_random(AS7038_GPIO_E, cfg);
    return err;
}

uint8_t as7038_init(void)
{
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
//	HAL_I2C_Init(&hi2c1);

//	HAL_Delay(5000);
//


    // Init structures
    memset(pox_samples, 0, POX_SAMPLES * sizeof(uint16_t));

    // Check if ic is available
    uint8_t id;
    uint8_t err  = as7038_read_random(AS7038_ID, &id);

    // Start osc and ldo
    ////    AS7038_CONTROL   (0x00);       7:5 -> Not used     4 ->  Enable I2C high speed    3 -> Not used   2-> Frequency  1 -> osc-en   0 -> ldo-en
    err = as7038_write_random(AS7038_CONTROL, 0x03);

//     Enable Sigref
    ////  AS7038_LED_CFG   (0x10);    7 -> Sigref-en   6 -> sigref-ecg-vol    5:4 -> sigref-ofe-vol   3 -> led4-en  2 -> led3-en  1 -> led2-en   0 -> led1-en
    err += as7038_write_random(AS7038_LED_CFG, 0x80);

////     LED config;
    err += as7038_led_current(AS7038_LED_1, AS7038_LED_CURRENT_35mA);
    err += as7038_led_current(AS7038_LED_2, AS7038_LED_CURRENT_35mA);
    err += as7038_led_current(AS7038_LED_3, AS7038_LED_CURRENT_35mA);
    err += as7038_led_current(AS7038_LED_4, AS7038_LED_CURRENT_35mA);

    err += as7038_config_gpio_output(AS7038_GPIO_0);

    err += as7038_activate_LEDs_LDO(AS7038_GPIO_0);

    if (id != AS7038_DEVICE_ID || err != HAL_OK)
    {
        return HAL_ERROR+9;
    }
    return err;
}



