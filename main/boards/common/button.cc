#include "button.h"

#include <esp_log.h>

static const char *TAG = "Button";

Button::Button(gpio_num_t gpio_num, button_type_t type)
    : gpio_num_(gpio_num)
{
    if (gpio_num_ == GPIO_NUM_NC) {
        return;
    }
    button_config_t button_config = {
        .type = type,
        .long_press_time = 1000,
        .short_press_time = 50,
        .gpio_button_config = {
            .gpio_num = gpio_ttl,
            .active_level = 0 },
    };
    if (BUTTON_TYPE_GPIO == type) {
        button_config.gpio_button_config.gpio_num = gpio_num;
        button_config.gpio_button_config.active_level = 0;
    } else if (BUTTON_TYPE_ADC == type) {
        button_config.adc_button_config.adc_channel = 0,
        button_config.adc_button_config.button_index = 0,
        button_config.adc_button_config.min = 100,
        button_config.adc_button_config.max = 400,
    }

    button_handle_ = iot_button_create(&button_config);
    if (button_handle_ == NULL) {
        ESP_LOGE(TAG, "Failed to create button handle");
        return;
    }
}

Button::Button(gpio_num_t gpio_adc, )
    : gpio_num_(gpio_adc),
      button_num_(numbers)
{
    if ((gpio_adc == GPIO_NUM_NC) || (0 == numbers)) {
        return;
    }

    button_config_t adc_btn_cfg = {
        .type = BUTTON_TYPE_ADC,
        .long_press_time = 1000,
        .short_press_time = 50,
        .adc_button_config = {

        },
    };
    button_handle_t button_handle_ = iot_button_create(&adc_btn_cfg);
    if (NULL == button_handle_) {
        ESP_LOGE(TAG, "Button create failed");
    }
}

Button::~Button()
{
    if (button_handle_ != NULL) {
        iot_button_delete(button_handle_);
    }
}

void Button::OnPress(std::function<void()> callback)
{
    if (button_handle_ == nullptr) {
        return;
    }
    on_press_ = callback;
    iot_button_register_cb(button_handle_, BUTTON_PRESS_DOWN, [](void *handle, void *usr_data) {
        Button* button = static_cast<Button*>(usr_data);
        if (button->on_press_) {
            button->on_press_();
        } }, this);
}

void Button::OnLongPress(std::function<void()> callback)
{
    if (button_handle_ == nullptr) {
        return;
    }
    on_long_press_ = callback;
    iot_button_register_cb(button_handle_, BUTTON_LONG_PRESS_START, [](void *handle, void *usr_data) {
        Button* button = static_cast<Button*>(usr_data);
        if (button->on_long_press_) {
            button->on_long_press_();
        } }, this);
}

void Button::OnClick(std::function<void()> callback)
{
    if (button_handle_ == nullptr) {
        return;
    }
    on_click_ = callback;
    iot_button_register_cb(button_handle_, BUTTON_SINGLE_CLICK, [](void *handle, void *usr_data) {
        Button* button = static_cast<Button*>(usr_data);
        if (button->on_click_) {
            button->on_click_();
        } }, this);
}

void Button::OnDoubleClick(std::function<void()> callback)
{
    if (button_handle_ == nullptr) {
        return;
    }
    on_double_click_ = callback;
    iot_button_register_cb(button_handle_, BUTTON_DOUBLE_CLICK, [](void *handle, void *usr_data) {
        Button* button = static_cast<Button*>(usr_data);
        if (button->on_double_click_) {
            button->on_double_click_();
        } }, this);
}
