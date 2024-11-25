#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE     16000
#define AUDIO_OUTPUT_SAMPLE_RATE    16000
#define AUDIO_DEFAULT_OUTPUT_VOLUME 80

#define AUDIO_INPUT_REFERENCE true

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_16
#define AUDIO_I2S_GPIO_WS   GPIO_NUM_45
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_9
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_10
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_8

#define AUDIO_CODEC_PA_PIN      BIT(0) /* TCA9554_GPIO_NUM_0 */
#define AUDIO_CODEC_I2C_SDA_PIN GPIO_NUM_17
#define AUDIO_CODEC_I2C_SCL_PIN GPIO_NUM_18
#define AUDIO_CODEC_ES8311_ADDR ES8311_CODEC_DEFAULT_ADDR // TODO 0x30
#define AUDIO_CODEC_ES7210_ADDR ES7210_CODEC_DEFAULT_ADDR // TODO 0x8*

#define BUILTIN_LED_GPIO        BIT(7) /* TCA9554_GPIO_NUM_7 */
#define BOOT_BUTTON_GPIO        GPIO_NUM_5
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC // TODO BUTTON ADC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC // TODO BUTTON ADC

/**
 * @brief LCD SCREEN Function Definition
 */
#define DISPLAY_BACKLIGHT_PIN  BIT(1)      // TCA9554_GPIO_NUM_1
#define DISPLAY_RST_GPIO       BIT(2)      // TCA9554_GPIO_NUM_2
#define DISPLAY_TOUCH_INT_GPIO BIT(4)      // TCA9554_GPIO_NUM_4
#define DISPLAY_CS_GPIO        GPIO_NUM_46 //
// DISPLAY SPI Pins
#define DISPLAY_DC_GPIO   GPIO_NUM_2
#define DISPLAY_CLK_GPIO  GPIO_NUM_1
#define DISPLAY_MOSI_GPIO GPIO_NUM_0
// The LCD pixel number in horizontal and vertical
#define DISPLAY_WIDTH    240
#define DISPLAY_HEIGHT   320
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true
#define DISPLAY_SWAP_XY  false

#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

#endif // _BOARD_CONFIG_H_
