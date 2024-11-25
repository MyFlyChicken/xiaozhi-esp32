#include "application.h"
#include "audio_codecs/box_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display/st7789_display.h"
#include "i2c_device.h"
#include "led.h"
#include "wifi_board.h"

#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>

#define TAG "esp32s3_du"

/* Register address */
#define INPUT_REG_ADDR     (0x00) /* 读取IO状态 */
#define OUTPUT_REG_ADDR    (0x01) /* 设置IO输出电平 */
#define DIRECTION_REG_ADDR (0x03) /* 设置IO方向，0输出 1输入 */

/* Default register value on power-up */
#define DIR_REG_DEFAULT_VAL (0xff)
#define OUT_REG_DEFAULT_VAL (0xff)

typedef enum {
    PCA9557_OUTPUT = 0, /*!< Output direction */
    PCA9557_INPUT,      /*!< Input dircetion */
} pca9557_dir_t;

/**
 * @brief 继承自I2cDevice
 *
 */
class Pca9557 : public I2cDevice {
public:
    /* 构造函数 */
    Pca9557(i2c_master_bus_handle_t i2c_bus, uint8_t addr)
        /* 初始化I2C */
        : I2cDevice(i2c_bus, addr)
    {
        /* 首先将IO扩展器全部设置为输入 */
        WriteReg(DIRECTION_REG_ADDR, DIR_REG_DEFAULT_VAL);
        /* 将板子用到的IO口设置为输出 */
        SetDir(DISPLAY_BACKLIGHT_PIN | DISPLAY_RST_GPIO | AUDIO_CODEC_PA_PIN | BUILTIN_LED_GPIO | DISPLAY_TOUCH_INT_GPIO, PCA9557_OUTPUT);
    }

    void SetDir(uint32_t mask, pca9557_dir_t dir)
    {
        uint8_t data, tmp;

        if (mask >= BIT64(8)) {
            ESP_LOGW(TAG, "mask out of range, bit higher than %d won't work", BIT64(8) - 1);
        }

        data = ReadReg(DIRECTION_REG_ADDR);
        tmp = data;
        for (int i = 0; i < 8; i++) {
            if (dir != ((tmp >> i) & dir)) {
                if (dir) {
                    data |= (1 << i);
                } else {
                    data &= ~(1 << i);
                }
            }
        }

        WriteReg(DIRECTION_REG_ADDR, data);
    }

    void SetOutputState(uint8_t bit, uint8_t level)
    {
        uint8_t data = ReadReg(OUTPUT_REG_ADDR);
        data = (data & ~(1 << bit)) | (level << bit);
        WriteReg(OUTPUT_REG_ADDR, data);
    }
};

class esp32s3_du : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    i2c_master_dev_handle_t pca9557_handle_;
    Button boot_button_;
    St7789Display *display_;
    Pca9557 *pca9557_;

    void InitializeI2c()
    {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));

        // Initialize PCA9557
        pca9557_ = new Pca9557(i2c_bus_, 0x20);
    }

    void InitializeSpi()
    {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_GPIO;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_GPIO;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons()
    {
        boot_button_.OnClick(
            [this]() { Application::GetInstance().ToggleChatState(); });
    }

    void InitializeSt7789Display()
    {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_GPIO;
        io_config.dc_gpio_num = DISPLAY_DC_GPIO;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.data_endian = LCD_RGB_DATA_ENDIAN_LITTLE;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);

        pca9557_->SetOutputState(DISPLAY_RST_GPIO, 0);
        pca9557_->SetOutputState(DISPLAY_BACKLIGHT_PIN, 0);
        pca9557_->SetOutputState(DISPLAY_TOUCH_INT_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        pca9557_->SetOutputState(DISPLAY_TOUCH_INT_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(6));
        pca9557_->SetOutputState(DISPLAY_RST_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(6));

        pca9557_->SetDir(DISPLAY_TOUCH_INT_GPIO, PCA9557_INPUT);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        //esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);

        display_ = new St7789Display(panel_io, panel, GPIO_NUM_NC,
                                     DISPLAY_BACKLIGHT_OUTPUT_INVERT, DISPLAY_WIDTH,
                                     DISPLAY_HEIGHT, DISPLAY_MIRROR_X,
                                     DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

public:
    esp32s3_du()
        : boot_button_(BOOT_BUTTON_GPIO)
    {
    }

    virtual void Initialize() override
    {
        ESP_LOGI(TAG, "Initializing esp32s3_du");
        InitializeI2c();
        InitializeSpi();
        InitializeSt7789Display();
        InitializeButtons();
        WifiBoard::Initialize();
    }

    virtual Led *GetBuiltinLed() override
    {
        static Led led(GPIO_NUM_NC);
        return &led;
    }

    virtual AudioCodec *GetAudioCodec() override
    {
        static BoxAudioCodec *audio_codec = nullptr;
        if (audio_codec == nullptr) {
            audio_codec = new BoxAudioCodec(
                i2c_bus_, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
                AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN, GPIO_NUM_NC,
                AUDIO_CODEC_ES8311_ADDR, AUDIO_CODEC_ES7210_ADDR,
                AUDIO_INPUT_REFERENCE);
            audio_codec->SetOutputVolume(AUDIO_DEFAULT_OUTPUT_VOLUME);
        }
        return audio_codec;
    }

    virtual Display *GetDisplay() override
    {
        return display_;
    }
};

DECLARE_BOARD(esp32s3_du);
