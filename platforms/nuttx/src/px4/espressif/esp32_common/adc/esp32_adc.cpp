#include <board_config.h>
#include <stdint.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>

// #include <esp32_adc.h>
#include <esp32_gpio.h>
#include "sens_reg.h"
#include "rtc_io_channel.h"
#include "rtc_cntl_reg.h"
#include "hardware/esp32_iomux.h"

const esp32_gpioMux_t esp32_gpioMux[GPIO_PIN_COUNT]={
    {0x44, 11, 11, 1},
    {0x88, -1, -1, -1},
    {0x40, 12, 12, 2},
    {0x84, -1, -1, -1},
    {0x48, 10, 10, 0},
    {0x6c, -1, -1, -1},
    {0x60, -1, -1, -1},
    {0x64, -1, -1, -1},
    {0x68, -1, -1, -1},
    {0x54, -1, -1, -1},
    {0x58, -1, -1, -1},
    {0x5c, -1, -1, -1},
    {0x34, 15, 15, 5},
    {0x38, 14, 14, 4},
    {0x30, 16, 16, 6},
    {0x3c, 13, 13, 3},
    {0x4c, -1, -1, -1},
    {0x50, -1, -1, -1},
    {0x70, -1, -1, -1},
    {0x74, -1, -1, -1},
    {0x78, -1, -1, -1},
    {0x7c, -1, -1, -1},
    {0x80, -1, -1, -1},
    {0x8c, -1, -1, -1},
    {0, -1, -1, -1},
    {0x24, 6, 18, -1}, //DAC1
    {0x28, 7, 19, -1}, //DAC2
    {0x2c, 17, 17, 7},
    {0, -1, -1, -1},
    {0, -1, -1, -1},
    {0, -1, -1, -1},
    {0, -1, -1, -1},
    {0x1c, 9, 4, 9},
    {0x20, 8, 5, 8},
    {0x14, 4, 6, -1},
    {0x18, 5, 7, -1},
    {0x04, 0, 0, -1},
    {0x08, 1, 1, -1},
    {0x0c, 2, 2, -1},
    {0x10, 3, 3, -1}
};

typedef struct {
    uint32_t reg;       /*!< Register of RTC pad, or 0 if not an RTC GPIO */
    uint32_t mux;       /*!< Bit mask for selecting digital pad or RTC pad */
    uint32_t func;      /*!< Shift of pad function (FUN_SEL) field */
    uint32_t ie;        /*!< Mask of input enable */
    uint32_t pullup;    /*!< Mask of pullup enable */
    uint32_t pulldown;  /*!< Mask of pulldown enable */
    uint32_t slpsel;    /*!< If slpsel bit is set, slpie will be used as pad input enabled signal in sleep mode */
    uint32_t slpie;     /*!< Mask of input enable in sleep mode */
    uint32_t hold;      /*!< Mask of hold enable */
    uint32_t hold_force;/*!< Mask of hold_force bit for RTC IO in RTC_CNTL_HOLD_FORCE_REG */
    uint32_t drv_v;     /*!< Mask of drive capability */
    uint32_t drv_s;     /*!< Offset of drive capability */
    int rtc_num;        /*!< RTC IO number, or -1 if not an RTC GPIO */
} rtc_gpio_desc_t;

const rtc_gpio_desc_t rtc_gpio_desc[GPIO_PIN_COUNT] = {
    {RTC_IO_TOUCH_PAD1_REG, RTC_IO_TOUCH_PAD1_MUX_SEL_M, RTC_IO_TOUCH_PAD1_FUN_SEL_S, RTC_IO_TOUCH_PAD1_FUN_IE_M, RTC_IO_TOUCH_PAD1_RUE_M, RTC_IO_TOUCH_PAD1_RDE_M, RTC_IO_TOUCH_PAD1_SLP_SEL_M, RTC_IO_TOUCH_PAD1_SLP_IE_M, RTC_IO_TOUCH_PAD1_HOLD_M, RTC_CNTL_TOUCH_PAD1_HOLD_FORCE_M, RTC_IO_TOUCH_PAD1_DRV_V, RTC_IO_TOUCH_PAD1_DRV_S, RTCIO_GPIO0_CHANNEL}, //0
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //1
    {RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_MUX_SEL_M, RTC_IO_TOUCH_PAD2_FUN_SEL_S, RTC_IO_TOUCH_PAD2_FUN_IE_M, RTC_IO_TOUCH_PAD2_RUE_M, RTC_IO_TOUCH_PAD2_RDE_M, RTC_IO_TOUCH_PAD2_SLP_SEL_M, RTC_IO_TOUCH_PAD2_SLP_IE_M, RTC_IO_TOUCH_PAD2_HOLD_M, RTC_CNTL_TOUCH_PAD2_HOLD_FORCE_M, RTC_IO_TOUCH_PAD2_DRV_V, RTC_IO_TOUCH_PAD2_DRV_S, RTCIO_GPIO2_CHANNEL}, //2
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //3
    {RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_MUX_SEL_M, RTC_IO_TOUCH_PAD0_FUN_SEL_S, RTC_IO_TOUCH_PAD0_FUN_IE_M, RTC_IO_TOUCH_PAD0_RUE_M, RTC_IO_TOUCH_PAD0_RDE_M, RTC_IO_TOUCH_PAD0_SLP_SEL_M, RTC_IO_TOUCH_PAD0_SLP_IE_M, RTC_IO_TOUCH_PAD0_HOLD_M,  RTC_CNTL_TOUCH_PAD0_HOLD_FORCE_M, RTC_IO_TOUCH_PAD0_DRV_V, RTC_IO_TOUCH_PAD0_DRV_S, RTCIO_GPIO4_CHANNEL}, //4
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //5
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //6
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //7
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //8
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //9
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //10
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //11
    {RTC_IO_TOUCH_PAD5_REG, RTC_IO_TOUCH_PAD5_MUX_SEL_M, RTC_IO_TOUCH_PAD5_FUN_SEL_S, RTC_IO_TOUCH_PAD5_FUN_IE_M, RTC_IO_TOUCH_PAD5_RUE_M, RTC_IO_TOUCH_PAD5_RDE_M, RTC_IO_TOUCH_PAD5_SLP_SEL_M, RTC_IO_TOUCH_PAD5_SLP_IE_M, RTC_IO_TOUCH_PAD5_HOLD_M, RTC_CNTL_TOUCH_PAD5_HOLD_FORCE_M, RTC_IO_TOUCH_PAD5_DRV_V, RTC_IO_TOUCH_PAD5_DRV_S, RTCIO_GPIO12_CHANNEL}, //12
    {RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_MUX_SEL_M, RTC_IO_TOUCH_PAD4_FUN_SEL_S, RTC_IO_TOUCH_PAD4_FUN_IE_M, RTC_IO_TOUCH_PAD4_RUE_M, RTC_IO_TOUCH_PAD4_RDE_M, RTC_IO_TOUCH_PAD4_SLP_SEL_M, RTC_IO_TOUCH_PAD4_SLP_IE_M, RTC_IO_TOUCH_PAD4_HOLD_M, RTC_CNTL_TOUCH_PAD4_HOLD_FORCE_M, RTC_IO_TOUCH_PAD4_DRV_V, RTC_IO_TOUCH_PAD4_DRV_S, RTCIO_GPIO13_CHANNEL}, //13
    {RTC_IO_TOUCH_PAD6_REG, RTC_IO_TOUCH_PAD6_MUX_SEL_M, RTC_IO_TOUCH_PAD6_FUN_SEL_S, RTC_IO_TOUCH_PAD6_FUN_IE_M, RTC_IO_TOUCH_PAD6_RUE_M, RTC_IO_TOUCH_PAD6_RDE_M, RTC_IO_TOUCH_PAD6_SLP_SEL_M, RTC_IO_TOUCH_PAD6_SLP_IE_M, RTC_IO_TOUCH_PAD6_HOLD_M, RTC_CNTL_TOUCH_PAD6_HOLD_FORCE_M, RTC_IO_TOUCH_PAD6_DRV_V, RTC_IO_TOUCH_PAD6_DRV_S, RTCIO_GPIO14_CHANNEL}, //14
    {RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_MUX_SEL_M, RTC_IO_TOUCH_PAD3_FUN_SEL_S, RTC_IO_TOUCH_PAD3_FUN_IE_M, RTC_IO_TOUCH_PAD3_RUE_M, RTC_IO_TOUCH_PAD3_RDE_M, RTC_IO_TOUCH_PAD3_SLP_SEL_M, RTC_IO_TOUCH_PAD3_SLP_IE_M, RTC_IO_TOUCH_PAD3_HOLD_M, RTC_CNTL_TOUCH_PAD3_HOLD_FORCE_M, RTC_IO_TOUCH_PAD3_DRV_V, RTC_IO_TOUCH_PAD3_DRV_S, RTCIO_GPIO15_CHANNEL}, //15
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //16
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //17
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //18
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //19
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //20
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //21
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //22
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //23
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //24
    {RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_MUX_SEL_M, RTC_IO_PDAC1_FUN_SEL_S, RTC_IO_PDAC1_FUN_IE_M, RTC_IO_PDAC1_RUE_M, RTC_IO_PDAC1_RDE_M, RTC_IO_PDAC1_SLP_SEL_M, RTC_IO_PDAC1_SLP_IE_M, RTC_IO_PDAC1_HOLD_M, RTC_CNTL_PDAC1_HOLD_FORCE_M, RTC_IO_PDAC1_DRV_V, RTC_IO_PDAC1_DRV_S, RTCIO_GPIO25_CHANNEL},                           //25
    {RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_MUX_SEL_M, RTC_IO_PDAC2_FUN_SEL_S, RTC_IO_PDAC2_FUN_IE_M, RTC_IO_PDAC2_RUE_M, RTC_IO_PDAC2_RDE_M, RTC_IO_PDAC2_SLP_SEL_M, RTC_IO_PDAC2_SLP_IE_M, RTC_IO_PDAC2_HOLD_M, RTC_CNTL_PDAC2_HOLD_FORCE_M, RTC_IO_PDAC2_DRV_V, RTC_IO_PDAC2_DRV_S, RTCIO_GPIO26_CHANNEL},                           //26
    {RTC_IO_TOUCH_PAD7_REG, RTC_IO_TOUCH_PAD7_MUX_SEL_M, RTC_IO_TOUCH_PAD7_FUN_SEL_S, RTC_IO_TOUCH_PAD7_FUN_IE_M, RTC_IO_TOUCH_PAD7_RUE_M, RTC_IO_TOUCH_PAD7_RDE_M, RTC_IO_TOUCH_PAD7_SLP_SEL_M, RTC_IO_TOUCH_PAD7_SLP_IE_M, RTC_IO_TOUCH_PAD7_HOLD_M, RTC_CNTL_TOUCH_PAD7_HOLD_FORCE_M, RTC_IO_TOUCH_PAD7_DRV_V, RTC_IO_TOUCH_PAD7_DRV_S, RTCIO_GPIO27_CHANNEL}, //27
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //28
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //29
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //30
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1},                                                                                                                                            //31
    {RTC_IO_XTAL_32K_PAD_REG, RTC_IO_X32P_MUX_SEL_M, RTC_IO_X32P_FUN_SEL_S, RTC_IO_X32P_FUN_IE_M, RTC_IO_X32P_RUE_M, RTC_IO_X32P_RDE_M, RTC_IO_X32P_SLP_SEL_M, RTC_IO_X32P_SLP_IE_M, RTC_IO_X32P_HOLD_M, RTC_CNTL_X32P_HOLD_FORCE_M, RTC_IO_X32P_DRV_V, RTC_IO_X32P_DRV_S, RTCIO_GPIO32_CHANNEL},                            //32
    {RTC_IO_XTAL_32K_PAD_REG, RTC_IO_X32N_MUX_SEL_M, RTC_IO_X32N_FUN_SEL_S, RTC_IO_X32N_FUN_IE_M, RTC_IO_X32N_RUE_M, RTC_IO_X32N_RDE_M, RTC_IO_X32N_SLP_SEL_M, RTC_IO_X32N_SLP_IE_M, RTC_IO_X32N_HOLD_M, RTC_CNTL_X32N_HOLD_FORCE_M, RTC_IO_X32N_DRV_V, RTC_IO_X32N_DRV_S, RTCIO_GPIO33_CHANNEL},                            //33
    {RTC_IO_ADC_PAD_REG, RTC_IO_ADC1_MUX_SEL_M, RTC_IO_ADC1_FUN_SEL_S, RTC_IO_ADC1_FUN_IE_M, 0, 0, RTC_IO_ADC1_SLP_SEL_M, RTC_IO_ADC1_SLP_IE_M, RTC_IO_ADC1_HOLD_M, RTC_CNTL_ADC1_HOLD_FORCE_M, 0, 0, RTCIO_GPIO34_CHANNEL},                                                                //34
    {RTC_IO_ADC_PAD_REG, RTC_IO_ADC2_MUX_SEL_M, RTC_IO_ADC2_FUN_SEL_S, RTC_IO_ADC2_FUN_IE_M, 0, 0, RTC_IO_ADC2_SLP_SEL_M, RTC_IO_ADC2_SLP_IE_M, RTC_IO_ADC2_HOLD_M, RTC_CNTL_ADC2_HOLD_FORCE_M, 0, 0, RTCIO_GPIO35_CHANNEL},                                                                //35
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE1_MUX_SEL_M, RTC_IO_SENSE1_FUN_SEL_S, RTC_IO_SENSE1_FUN_IE_M, 0, 0, RTC_IO_SENSE1_SLP_SEL_M, RTC_IO_SENSE1_SLP_IE_M, RTC_IO_SENSE1_HOLD_M, RTC_CNTL_SENSE1_HOLD_FORCE_M, 0, 0, RTCIO_GPIO36_CHANNEL},                                                      //36
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE2_MUX_SEL_M, RTC_IO_SENSE2_FUN_SEL_S, RTC_IO_SENSE2_FUN_IE_M, 0, 0, RTC_IO_SENSE2_SLP_SEL_M, RTC_IO_SENSE2_SLP_IE_M, RTC_IO_SENSE2_HOLD_M, RTC_CNTL_SENSE2_HOLD_FORCE_M, 0, 0, RTCIO_GPIO37_CHANNEL},                                                      //37
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE3_MUX_SEL_M, RTC_IO_SENSE3_FUN_SEL_S, RTC_IO_SENSE3_FUN_IE_M, 0, 0, RTC_IO_SENSE3_SLP_SEL_M, RTC_IO_SENSE3_SLP_IE_M, RTC_IO_SENSE3_HOLD_M, RTC_CNTL_SENSE3_HOLD_FORCE_M, 0, 0, RTCIO_GPIO38_CHANNEL},                                                       //38
    {RTC_IO_SENSOR_PADS_REG, RTC_IO_SENSE4_MUX_SEL_M, RTC_IO_SENSE4_FUN_SEL_S, RTC_IO_SENSE4_FUN_IE_M, 0, 0, RTC_IO_SENSE4_SLP_SEL_M, RTC_IO_SENSE4_SLP_IE_M, RTC_IO_SENSE4_HOLD_M, RTC_CNTL_SENSE4_HOLD_FORCE_M, 0, 0, RTCIO_GPIO39_CHANNEL},                                                      //39
};

#define ESP_REG(addr) *((volatile uint32_t *)(addr))

static void __setPinModeAnalog(uint8_t pin);
static void __analogInit(void);

bool adcAttachPin(uint8_t pin)
{

    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        return false;//not adc pin
    }

    int8_t pad = digitalPinToTouchChannel(pin);
    if(pad >= 0){
        uint32_t touch = READ_PERI_REG(SENS_SAR_TOUCH_ENABLE_REG);
        if(touch & (1 << pad)){
            touch &= ~((1 << (pad + SENS_TOUCH_PAD_OUTEN2_S))
                    | (1 << (pad + SENS_TOUCH_PAD_OUTEN1_S))
                    | (1 << (pad + SENS_TOUCH_PAD_WORKEN_S)));
            WRITE_PERI_REG(SENS_SAR_TOUCH_ENABLE_REG, touch);
        }
    } else if(pin == 25){
        CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC | RTC_IO_PDAC1_DAC_XPD_FORCE);//stop dac1
    } else if(pin == 26){
        CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC2_REG, RTC_IO_PDAC2_XPD_DAC | RTC_IO_PDAC2_DAC_XPD_FORCE);//stop dac2
    }

    __setPinModeAnalog(pin);

    __analogInit();
    return true;
}

static void __setPinModeAnalog(uint8_t pin)
{
    uint32_t rtc_reg = rtc_gpio_desc[pin].reg;
    {
        if(!rtc_reg) {
            return;//not rtc pin
        }
        //lock rtc
        uint32_t reg_val = ESP_REG(rtc_reg);
        if(reg_val & rtc_gpio_desc[pin].mux){
            return;//already in adc mode
        }
        reg_val &= ~(
                (RTC_IO_TOUCH_PAD1_FUN_SEL_V << rtc_gpio_desc[pin].func)
                |rtc_gpio_desc[pin].ie
                |rtc_gpio_desc[pin].pullup
                |rtc_gpio_desc[pin].pulldown);
        ESP_REG(RTC_GPIO_ENABLE_W1TC_REG) = (1 << (rtc_gpio_desc[pin].rtc_num + RTC_GPIO_ENABLE_W1TC_S));
        ESP_REG(rtc_reg) = reg_val | rtc_gpio_desc[pin].mux;
        //unlock rtc
        ESP_REG(DR_REG_IO_MUX_BASE + esp32_gpioMux[pin].reg) = ((uint32_t)2 << MCU_SEL_S) | ((uint32_t)2 << FUN_DRV_S) | FUN_IE;
        return;
    }

    //RTC pins PULL settings
    if(rtc_reg) {
        //lock rtc
        ESP_REG(rtc_reg) = ESP_REG(rtc_reg) & ~(rtc_gpio_desc[pin].mux);
        ESP_REG(rtc_reg) = ESP_REG(rtc_reg) & ~(rtc_gpio_desc[pin].pullup | rtc_gpio_desc[pin].pulldown);
        //unlock rtc
    }

    uint32_t pinFunction = 0;

    pinFunction |= ((uint32_t)2 << FUN_DRV_S);//what are the drivers?
    pinFunction |= FUN_IE;//input enable but required for output as well?

    pinFunction |= ((uint32_t)(0xC0 >> 5) << MCU_SEL_S);

    ESP_REG(DR_REG_IO_MUX_BASE + esp32_gpioMux[pin].reg) = pinFunction;

//     GPIO.pin[pin].val = pinControl;
}

static adc_attenuation_t __analogAttenuation = ADC_11db;//11db
static uint8_t __analogWidth = 3;//12 bits
static uint8_t __analogCycles = 8;
static uint8_t __analogSamples = 0;//1 sample
static uint8_t __analogClockDiv = 1;

// Width of returned answer ()
static uint8_t __analogReturnedWidth = 12;

void __analogSetWidth(uint8_t bits){
    if(bits < 9){
        bits = 9;
    } else if(bits > 12){
        bits = 12;
    }
    __analogReturnedWidth = bits;
    __analogWidth = bits - 9;
    SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR1_BIT_WIDTH, __analogWidth, SENS_SAR1_BIT_WIDTH_S);
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_BIT, __analogWidth, SENS_SAR1_SAMPLE_BIT_S);

    SET_PERI_REG_BITS(SENS_SAR_START_FORCE_REG, SENS_SAR2_BIT_WIDTH, __analogWidth, SENS_SAR2_BIT_WIDTH_S);
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_BIT, __analogWidth, SENS_SAR2_SAMPLE_BIT_S);
}

void __analogSetCycles(uint8_t cycles){
    __analogCycles = cycles;
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_CYCLE, __analogCycles, SENS_SAR1_SAMPLE_CYCLE_S);
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_CYCLE, __analogCycles, SENS_SAR2_SAMPLE_CYCLE_S);
}

void __analogSetSamples(uint8_t samples){
    if(!samples){
        return;
    }
    __analogSamples = samples - 1;
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_SAMPLE_NUM, __analogSamples, SENS_SAR1_SAMPLE_NUM_S);
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_SAMPLE_NUM, __analogSamples, SENS_SAR2_SAMPLE_NUM_S);
}

void __analogSetClockDiv(uint8_t clockDiv){
    if(!clockDiv){
        return;
    }
    __analogClockDiv = clockDiv;
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL_REG, SENS_SAR1_CLK_DIV, __analogClockDiv, SENS_SAR1_CLK_DIV_S);
    SET_PERI_REG_BITS(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_CLK_DIV, __analogClockDiv, SENS_SAR2_CLK_DIV_S);
}

void __analogSetAttenuation(adc_attenuation_t attenuation)
{
    __analogAttenuation = (adc_attenuation_t)(attenuation & 3);
    uint32_t att_data = 0;
    int i = 10;
    while(i--){
        att_data |= __analogAttenuation << (i * 2);
    }
    WRITE_PERI_REG(SENS_SAR_ATTEN1_REG, att_data & 0xFFFF);//ADC1 has 8 channels
    WRITE_PERI_REG(SENS_SAR_ATTEN2_REG, att_data);
}

static void __analogInit(void)
{
    static bool initialized = false;
    if(initialized){
        return;
    }

    __analogSetAttenuation(__analogAttenuation);
    __analogSetCycles(__analogCycles);
    __analogSetSamples(__analogSamples + 1);//in samples
    __analogSetClockDiv(__analogClockDiv);
    __analogSetWidth(__analogWidth + 9);//in bits

    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL_REG, SENS_SAR1_DATA_INV);
    SET_PERI_REG_MASK(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DATA_INV);

    SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_FORCE_M); //SAR ADC1 controller (in RTC) is started by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD_FORCE_M); //SAR ADC1 pad enable bitmap is controlled by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_FORCE_M); //SAR ADC2 controller (in RTC) is started by SW
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD_FORCE_M); //SAR ADC2 pad enable bitmap is controlled by SW

    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_M); //force XPD_SAR=0, use XPD_FSM
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); //force XPD_AMP=0

    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_CTRL_REG, 0xfff << SENS_AMP_RST_FB_FSM_S);  //clear FSM
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
    while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_MEAS_STATUS_S) != 0); //wait det_fsm==

    initialized = true;
}

static bool __adcStartPin(uint8_t pin){

    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        return false;//not adc pin
    }

    if(channel > 9){
        channel -= 10;
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD, (1 << channel), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
    }
    return true;
}

// static bool __adcBusyPin(uint8_t pin){

//     int8_t channel = digitalPinToAnalogChannel(pin);
//     if(channel < 0){
//         return false;//not adc pin
//     }

//     if(channel > 7){
//         return (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0);
//     }
//     return (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0);
// }

static uint16_t __adcEndPin(uint8_t pin)
{

    uint16_t value = 0;
    int8_t channel = digitalPinToAnalogChannel(pin);
    if(channel < 0){
        return 0;//not adc pin
    }
    if(channel > 7){
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0); //wait for conversion
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0); //wait for conversion
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    // Shift result if necessary
    uint8_t from = __analogWidth + 9;
    if (from == __analogReturnedWidth) {
        return value;
    }
    if (from > __analogReturnedWidth) {
        return value >> (from - __analogReturnedWidth);
    }
    return value << (__analogReturnedWidth - from);
}

static bool __adcStart(uint8_t channel){

    if(channel > 9){
        channel -= 10;
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD, (1 << channel), SENS_SAR2_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
    } else {
        CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
        SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
    }
    return true;
}

// static bool __adcBusy(uint8_t channel){

//     if(channel > 7){
//         return (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0);
//     }
//     return (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0);
// }

static uint16_t __adcEnd(uint8_t channel)
{
    uint16_t value = 0;

    if(channel > 7){
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0); //wait for conversion
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
    } else {
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0); //wait for conversion
        value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    // Shift result if necessary
    uint8_t from = __analogWidth + 9;
    if (from == __analogReturnedWidth) {
        return value;
    }
    if (from > __analogReturnedWidth) {
        return value >> (from - __analogReturnedWidth);
    }
    return value << (__analogReturnedWidth - from);
}


uint16_t analogReadPin(uint8_t pin)
{
    if(!__adcStartPin(pin)){
        return 0;
    }
    return __adcEndPin(pin);
}

uint16_t analogRead(uint8_t channel)
{
    if(!__adcStart(channel)){
        return 0;
    }
    return __adcEnd(channel);
}
