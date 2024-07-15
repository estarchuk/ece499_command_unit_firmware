#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rotary_encoder.h"
#include <esp_err.h>
#include <esp_log.h>
#include "driver/i2c_master.h"
#include "sdkconfig.h"
#include "rom/uart.h"
#include "rom/gpio.h"
#include "smbus.h"
#include "i2c-lcd1602.h"
#include <string>
#include <cstring>
#include <array>
#include <esp_timer.h>
#include "isobus/hardware_integration/twai_plugin.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/can_partnered_control_function.hpp"
#include <iostream>
#include <hal/uart_types.h>
#include <driver/uart.h>

#define TAG "app"

// LCD2004
#define LCD_NUM_ROWS               4
#define LCD_NUM_COLUMNS            40
#define LCD_NUM_VISIBLE_COLUMNS    20

//#define USE_STDIN  1
#undef USE_STDIN

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        3
#define I2C_MASTER_SCL_IO        2

// Names so we don't have to constantly look at GPIO pin assignments
#define led_0 GPIO_NUM_4
#define led_1 GPIO_NUM_5
#define led_2 GPIO_NUM_6
#define led_3 GPIO_NUM_7
#define button GPIO_NUM_8
#define encoder_a GPIO_NUM_21
#define encoder_b GPIO_NUM_20
#define isobus_tx GPIO_NUM_10
#define isobus_rx GPIO_NUM_9

// I enabled half steps because it gave a more reliable input
// we may be able to set it false with our debouncing

#define ENABLE_HALF_STEPS true  // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT          0     // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION    false  // Set to true to reverse the clockwise/counterclockwise sense


// Global variables
rotary_encoder_state_t state = { 0 };
rotary_encoder_info_t info = { GPIO_NUM_0 };
i2c_lcd1602_info_t * lcd_info;
QueueHandle_t event_queue = rotary_encoder_create_queue();

int led_state_0 = 0;
int led_state_1 = 0;
int led_state_2 = 0;
int led_state_3 = 0;

int button_flag = 0;

// i2c stuff

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    i2c_param_config(i2c_port_t(i2c_master_port), &conf);
    i2c_driver_install(i2c_port_t(i2c_master_port), conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}

// More i2c stuff. The other two comments are from the LCD library
// uart_rx_one_char_block() causes a watchdog trigger, so use the non-blocking
// uart_rx_one_char() and delay briefly to reset the watchdog.
// Might not even need this, this function is never used and was an example from the library anyway
static uint8_t _wait_for_user(void)
{
    uint8_t c = 0;

#ifdef USE_STDIN
    while (!c)
    {
       ETS_STATUS s = uart_rx_one_char(&c);
       if (s == OK) {
          printf("%c", c);
       }
       vTaskDelay(1);
    }
#else
    vTaskDelay(1000 / portTICK_PERIOD_MS);
#endif
    return c;
}

extern "C" void lcd_setup(void){
    // Set up I2C
    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = 0x27;

    // Set up the SMBus
    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_PERIOD_MS));

    // Set up the LCD1602 device with backlight off
    lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));
}

// Button ISR, only sets a flag. Debounce handled in hardware.
static void _isr_button(void * args){

    button_flag = 1;

}

void gpio_setup(void){
    // Reset GPIO pins, preventing unintended behaviour
    gpio_reset_pin(led_0);
    gpio_reset_pin(led_1);
    gpio_reset_pin(led_2);
    gpio_reset_pin(led_3);
    gpio_reset_pin(button);

    // Set GPIO pin direction
    gpio_set_direction(led_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(led_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(led_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(led_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(button, GPIO_MODE_INPUT);

    // Set initial LED status (off)
    gpio_set_level(led_0, 0);
    gpio_set_level(led_1, 0);
    gpio_set_level(led_2, 0);
    gpio_set_level(led_3, 0);

}

void rotary_encoder_setup(void){

    ESP_ERROR_CHECK(rotary_encoder_init(&info, encoder_a, encoder_b));
    ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS));
    
    #ifdef FLIP_DIRECTION
        ESP_ERROR_CHECK(rotary_encoder_flip_direction(&info));
    #endif

    ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue));

}

void interrupt_setup(void){

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    // Interrupt type needs to be high level, rising edge doesn't work sometimes
    gpio_set_intr_type(button, GPIO_INTR_LOW_LEVEL);
    gpio_isr_handler_add(button, _isr_button, NULL);

}

// This might not be needed on the command unit
void propa_callback(const isobus::CANMessage &CANMessage, void *){

    // If a message is received, dump it to the console
    std::cout << CANMessage.get_data_length() << std::endl;
}

// There is no guarantee anything works in here at this point (10 July 2024)
void isobus_setup(void){

    twai_general_config_t twaiConfig = TWAI_GENERAL_CONFIG_DEFAULT(isobus_tx, isobus_rx, TWAI_MODE_NORMAL);
    twai_timing_config_t twaiTiming = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t twaiFilter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    std::shared_ptr<isobus::CANHardwarePlugin> canDriver = std::make_shared<isobus::TWAIPlugin>(&twaiConfig, &twaiTiming, &twaiFilter);
    std::shared_ptr<isobus::InternalControlFunction> myECU = nullptr; // A pointer to hold our InternalControlFunction


    isobus::CANHardwareInterface::set_number_of_can_channels(1);
    isobus::CANHardwareInterface::assign_can_channel_frame_handler(0, canDriver);
    isobus::CANHardwareInterface::set_periodic_update_interval(10); // Default is 4ms, but we need to adjust this for default ESP32 tick rate of 100Hz

    if (!isobus::CANHardwareInterface::start() || !canDriver->get_is_valid())
    {
        ESP_LOGE("AgIsoStack", "Failed to start hardware interface, the CAN driver might be invalid");
    }

    isobus::NAME TestDeviceNAME(0);

    //! Consider customizing some of these fields, like the function code, to be representative of your device
    TestDeviceNAME.set_arbitrary_address_capable(true);
    TestDeviceNAME.set_industry_group(1);
    TestDeviceNAME.set_device_class(0);
    TestDeviceNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::RateControl));
    TestDeviceNAME.set_identity_number(2);
    TestDeviceNAME.set_ecu_instance(0);
    TestDeviceNAME.set_function_instance(0);
    TestDeviceNAME.set_device_class_instance(0);
    TestDeviceNAME.set_manufacturer_code(1407);
    auto TestInternalECU = isobus::CANNetworkManager::CANNetwork.create_internal_control_function(TestDeviceNAME, 0);

    isobus::CANNetworkManager::CANNetwork.add_global_parameter_group_number_callback(0xEF00, propa_callback, nullptr);

    std::array<std::uint8_t, isobus::CAN_DATA_LENGTH> messageData = {1}; // Data is just all ones

    isobus::CANNetworkManager::CANNetwork.send_can_message(0xEF00, messageData.data(), isobus::CAN_DATA_LENGTH, myECU);

}

extern "C" void app_main(void)
{

    gpio_setup();
    lcd_setup();
    interrupt_setup();
    rotary_encoder_setup();
    //isobus_setup();


    const uart_port_t uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, isobus_tx, isobus_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, uart_buffer_size, \
                                            uart_buffer_size, 10, &uart_queue, 0));
    
    while (1)
    {
        // Move this?
        rotary_encoder_event_t event = { 0 };

        // Will see if I can remove this
        if (xQueueReceive(event_queue, &event, 10) == pdTRUE)
        {
            //printf("Event: position %ld, direction %s\n", event.state.position,
            //         event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "CW" : "CCW") : "NOT_SET");
        } else {
            // Poll current position and direction
            // This is default from the rotary encoder library, I want to get rid of it
            // but couldn't figure it out in time
            ESP_ERROR_CHECK(rotary_encoder_get_state(&info, &state));
        }

        // For screen debugging mostly, super inefficient but I wanted a cursor
        for(int i = 0; i < 4; i++){
            i2c_lcd1602_move_cursor(lcd_info, 17, i);
            i2c_lcd1602_write_char(lcd_info, ' ');
        }
        
        switch (state.position){
            case 0:
                i2c_lcd1602_move_cursor(lcd_info, 17, 0);
                i2c_lcd1602_write_char(lcd_info, '*');
                break;
            case 1:
                i2c_lcd1602_move_cursor(lcd_info, 17, 1);
                i2c_lcd1602_write_char(lcd_info, '*');
                break;
            case 2:
                i2c_lcd1602_move_cursor(lcd_info, 17, 2);
                i2c_lcd1602_write_char(lcd_info, '*');
                break;
            case 3:
                i2c_lcd1602_move_cursor(lcd_info, 17, 3);
                i2c_lcd1602_write_char(lcd_info, '*');
                break;
        }

        // If the button has been pressed, toggle LED state
        // This logic is fine and can probably stay here
        if (button_flag == 1) {
            button_flag = 0;
            int test_int[1];
            test_int[0] = 0;

            switch (state.position) {
                case 0:
                    gpio_set_level(led_0, !led_state_0);
                    led_state_0 = !led_state_0;
                    test_int[0] = 0;
                    uart_write_bytes(uart_num, (const int*)test_int, sizeof(int));
                    break;
                case 1:
                    gpio_set_level(led_1, !led_state_1);
                    led_state_1 = !led_state_1;
                    test_int[0] = 1;
                    uart_write_bytes(uart_num, (const int*)test_int, sizeof(int));
                    break;
                case 2:
                    gpio_set_level(led_2, !led_state_2);
                    led_state_2 = !led_state_2;
                    test_int[0] = 2;
                    uart_write_bytes(uart_num, (const int*)test_int, sizeof(int));
                    break;
                case 3:
                    gpio_set_level(led_3, !led_state_3);
                    led_state_3 = !led_state_3;
                    test_int[0] = 3;
                    uart_write_bytes(uart_num, (const int*)test_int, sizeof(int));
                    break;
            }
        }
        
        // Super disgusting display code, should show each LED's status
        // 0 being off, 1 being on
        // This is gross, will try to relegate to a function out of the way
        std::string led_0_status, led_1_status, led_2_status, led_3_status;
        led_0_status = "LED 0 status: ";
        led_0_status.append(std::to_string(led_state_0));
        led_1_status = "LED 1 status: ";
        led_1_status.append(std::to_string(led_state_1));
        led_2_status = "LED 2 status: ";
        led_2_status.append(std::to_string(led_state_2));
        led_3_status = "LED 3 status: ";
        led_3_status.append(std::to_string(led_state_3));
        char * led_0_out = new char [led_0_status.length() + 1];
        char * led_1_out = new char [led_0_status.length() + 1];
        char * led_2_out = new char [led_0_status.length() + 1];
        char * led_3_out = new char [led_0_status.length() + 1];

        std::strcpy(led_0_out, led_0_status.c_str());
        std::strcpy(led_1_out, led_1_status.c_str());
        std::strcpy(led_2_out, led_2_status.c_str());
        std::strcpy(led_3_out, led_3_status.c_str());
        i2c_lcd1602_move_cursor(lcd_info, 0, 0);
        i2c_lcd1602_write_string(lcd_info, led_0_out);
        i2c_lcd1602_move_cursor(lcd_info, 0, 1);
        i2c_lcd1602_write_string(lcd_info, led_1_out);
        i2c_lcd1602_move_cursor(lcd_info, 0, 2);
        i2c_lcd1602_write_string(lcd_info, led_2_out);
        i2c_lcd1602_move_cursor(lcd_info, 0, 3);
        i2c_lcd1602_write_string(lcd_info, led_3_out);

    }
}