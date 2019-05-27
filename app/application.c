#include <application.h>
#include <bc_servo.h>

#include "at.h"

// LED instance
bc_led_t led;

// Button instance
bc_button_t button;

bc_servo_t servo;

bc_module_relay_t relay_module;

bc_soil_sensor_t soil_sensor;
bc_soil_sensor_sensor_t sensors[5];

void servo_set_angle(uint8_t angle);
void lora_send();

bc_led_t gps_led_r;
bc_led_t gps_led_g;

bc_led_t led_lcd_red;
bc_led_t led_lcd_green;

// LoRa instance
bc_cmwx1zzabz_t lora;

float voltage;

bc_module_gps_position_t position;

float temperature = NAN;
int moisture;

char deveui[20] = "...";
char devaddr[20] = "...";

bool valve_opened = false;

int top;

void screen_clear(void)
{
    bc_module_lcd_clear();
    bc_module_lcd_draw_string(2, 2, "BigClown.com LoRa", true);
    bc_module_lcd_draw_line(0, 17, 127, 17, true);

    top = 20;
}

void screen_append(char *s)
{
    //bc_log_info("APP: Append to screen: %s", s);

    bc_module_lcd_draw_string(2, top, s, true);

    top += 13;
}

void screen_update(void)
{
    bc_module_lcd_update();
}

void valve_open(bool open)
{
    valve_opened = open;

    if (open)
    {
        servo_set_angle(80);
        bc_led_set_mode(&led, BC_LED_MODE_OFF);
    }
    else
    {
        servo_set_angle(170);
        bc_led_set_mode(&led, BC_LED_MODE_ON);
    }
}

void button_event_handler(bc_button_t *self, bc_button_event_t event, void *event_param)
{

    static bool state = false;

    if (event == BC_BUTTON_EVENT_CLICK)
    {
        valve_open(state);

        bc_led_pulse(&led_lcd_green, 100);

        state = !state;
    }

    if (event == BC_BUTTON_EVENT_HOLD)
    {
        bc_led_pulse(&led, 500);

        lora_send();
    }

    // Logging in action
    bc_log_info("Button event handler - event: %i", event);
}

bc_scheduler_task_id_t servo_task_id;

void servo_task_stop(void *param)
{
    bc_module_relay_set_state(&relay_module, false);

    //bc_scheduler_unregister(servo_task_id);
}

void servo_set_angle(uint8_t angle)
{
    bc_module_relay_set_state(&relay_module, true);

    bc_servo_set_angle(&servo, angle);

    bc_scheduler_plan_relative(servo_task_id, 2000);
}


void lora_send()
{
    uint8_t buffer[14];

    buffer[0] = 0x01;

    int16_t temperature_i16 = (int16_t) (temperature * 10.f);

    buffer[1] = temperature_i16 >> 8;
    buffer[2] = temperature_i16;

    buffer[3] = moisture;

    buffer[4] = valve_opened;

    int32_t lat = position.latitude * 10000;

    buffer[5] = (lat >> 24) & 0xFF;
    buffer[6] = (lat >> 16) & 0xFF;
    buffer[7] = (lat >> 8) & 0xFF;
    buffer[8] = (lat >> 0) & 0xFF;

    int32_t lon = position.longitude * 10000;

    buffer[9] = (lon >> 24) & 0xFF;
    buffer[10] = (lon >> 16) & 0xFF;
    buffer[11] = (lon >> 8) & 0xFF;
    buffer[12] = (lon >> 0) & 0xFF;

    buffer[13] = ceil(voltage * 10.f);

    bc_cmwx1zzabz_send_message(&lora, buffer, sizeof(buffer));




}

void soil_sensor_event_handler(bc_soil_sensor_t *self, uint64_t device_address, bc_soil_sensor_event_t event, void *event_param)
{
    if (event == BC_SOIL_SENSOR_EVENT_UPDATE)
    {
        if (!bc_soil_sensor_get_temperature_celsius(self, device_address, &temperature))
        {
            bc_log_error("bc_soil_sensor_get_temperature_celsius");

            return;
        }

        //bc_log_info("soil-sensor temperature %llx %.1f °C", device_address, temperature);

        if (!bc_soil_sensor_get_moisture(self, device_address, &moisture))
        {
            bc_log_error("bc_soil_sensor_get_moisture");

            return;
        }

        //bc_log_info("soil-sensor moisture %llx %s %d %%", device_address, bc_soil_sensor_get_label(self, device_address), moisture);
    }
    else if (event == BC_SOIL_SENSOR_EVENT_ERROR)
    {
        bc_log_error("BC_SOIL_SENSOR_EVENT_ERROR");
    }
}



void lora_callback(bc_cmwx1zzabz_t *self, bc_cmwx1zzabz_event_t event, void *event_param)
{
    if (event == BC_CMWX1ZZABZ_EVENT_READY)
    {

        bc_cmwx1zzabz_get_deveui(&lora, (char*)deveui);
        bc_cmwx1zzabz_get_devaddr(&lora, (char*)devaddr);

        bc_log_debug("deveui: %s", deveui);
        bc_log_debug("devaddr: %s", devaddr);

        bc_led_pulse(&led, 50);
    }

    if (event == BC_CMWX1ZZABZ_EVENT_ERROR)
    {
        bc_led_set_mode(&led, BC_LED_MODE_BLINK_FAST);
        bc_log_debug("Modem ERROR");
    }

    if (event == BC_CMWX1ZZABZ_EVENT_JOIN_ERROR)
    {
        bc_log_debug("Join ERROR");
    }

    if (event == BC_CMWX1ZZABZ_EVENT_JOIN_SUCCESS)
    {
        bc_log_debug("Join OK");
    }

    if (event == BC_CMWX1ZZABZ_EVENT_SEND_MESSAGE_START)
    {
        bc_led_set_mode(&led, BC_LED_MODE_ON);
        bc_led_set_mode(&led_lcd_red, BC_LED_MODE_ON);
        bc_log_debug("Sending start");
    }

    if (event == BC_CMWX1ZZABZ_EVENT_SEND_MESSAGE_DONE)
    {
        bc_led_set_mode(&led, BC_LED_MODE_OFF);
        bc_led_set_mode(&led_lcd_red, BC_LED_MODE_OFF);
        bc_log_debug("Sending done");
    }

    if (event == BC_CMWX1ZZABZ_EVENT_MESSAGE_RECEIVED)
    {
        volatile uint8_t port = bc_cmwx1zzabz_get_received_message_port(self);
        volatile uint32_t length = bc_cmwx1zzabz_get_received_message_length(self);

        uint8_t msg_buffer[51];
        volatile uint32_t len = bc_cmwx1zzabz_get_received_message_data(self, msg_buffer, sizeof(msg_buffer));

        bc_led_pulse(&led, 1500);

        if (msg_buffer[0])
        {
            bc_led_pulse(&led_lcd_green, 2000);
        }
        else
        {
            bc_led_pulse(&led_lcd_red, 2000);
        }

        valve_open(msg_buffer[0]);

        bc_log_debug("Message received: 0x%02X", msg_buffer[0]);

        (void) len;
        (void) length;
        (void) port;
    }
}


void gps_module_event_handler(bc_module_gps_event_t event, void *event_param)
{
    if (event == BC_MODULE_GPS_EVENT_START)
    {
        bc_log_info("APP: Event BC_MODULE_GPS_EVENT_START");

        //bc_led_set_mode(&gps_led_g, BC_LED_MODE_ON);
    }
    else if (event == BC_MODULE_GPS_EVENT_STOP)
    {
        bc_log_info("APP: Event BC_MODULE_GPS_EVENT_STOP");

        //bc_led_set_mode(&gps_led_g, BC_LED_MODE_OFF);
    }
    else if (event == BC_MODULE_GPS_EVENT_UPDATE)
    {
        bc_led_pulse(&gps_led_g, 25);

        screen_clear();

        char buffer[64];

        bc_module_gps_time_t time;

        if (bc_module_gps_get_time(&time))
        {
            snprintf(buffer, sizeof(buffer),
                "Date: %04d-%02d-%02d",
                time.year+2000, time.month, time.day);

            screen_append(buffer);

            snprintf(buffer, sizeof(buffer),
                "Time: %02d:%02d:%02d",
                time.hours, time.minutes, time.seconds);

            screen_append(buffer);
        }

        if (bc_module_gps_get_position(&position))
        {
            snprintf(buffer, sizeof(buffer),
                "Lat: %03.5f", position.latitude);

            screen_append(buffer);

            snprintf(buffer, sizeof(buffer),
                "Lon: %03.5f", position.longitude);

            screen_append(buffer);
        }

        bc_module_gps_altitude_t altitude;

        if (bc_module_gps_get_altitude(&altitude))
        {
            snprintf(buffer, sizeof(buffer),
                "Altitude: %.1f %c",
                altitude.altitude, tolower(altitude.units));

        }

        bc_module_gps_quality_t quality;

        if (bc_module_gps_get_quality(&quality))
        {

            snprintf(buffer, sizeof(buffer),
                "Satellites: %d",
                quality.satellites_tracked);

            screen_append(buffer);
        }

        snprintf(buffer, sizeof(buffer), "Soil: %d%%, %.1fC", moisture, temperature);
        screen_append(buffer);

        snprintf(buffer, sizeof(buffer), "Valve opened: %c", (int)('0' + (char)valve_opened));
        screen_append(buffer);

        snprintf(buffer, sizeof(buffer), "Battery voltage: %.1f V", voltage);
        screen_append(buffer);

        screen_update();

        bc_module_gps_invalidate();
    }
    else if (event == BC_MODULE_GPS_EVENT_ERROR)
    {
        bc_log_info("APP: Event BC_MODULE_GPS_EVENT_ERROR");
    }
}


void battery_event_handler(bc_module_battery_event_t event, void *event_param)
{
    if (event == BC_MODULE_BATTERY_EVENT_UPDATE)
    {
        bc_module_battery_get_voltage(&voltage);
        bc_log_debug("Batt Voltage: %f", voltage);
    }
}

void application_init(void)
{
    // Initialize logging
    bc_log_init(BC_LOG_LEVEL_DUMP, BC_LOG_TIMESTAMP_ABS);

    // Initialize battery
    bc_module_battery_init();
    bc_module_battery_set_event_handler(battery_event_handler, NULL);
    bc_module_battery_set_update_interval(5 * 60 * 1000);

    // Initialize LED
    bc_led_init(&led, BC_GPIO_LED, false, false);
    bc_led_pulse(&led, 1000);

    // Initialize button
    bc_button_init(&button, BC_GPIO_BUTTON, BC_GPIO_PULL_DOWN, false);
    bc_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize soil sensor
    bc_soil_sensor_init_multiple(&soil_sensor, sensors, 5);
    bc_soil_sensor_set_event_handler(&soil_sensor, soil_sensor_event_handler, NULL);
    bc_soil_sensor_set_update_interval(&soil_sensor, 1000);

    // Initialize GPS
    if (!bc_module_gps_init())
    {
        bc_log_error("APP: GPS Module initialization failed");
    }
    else
    {
        bc_module_gps_set_event_handler(gps_module_event_handler, NULL);
        bc_module_gps_start();
    }

    // Initialize LCD Module
    bc_module_lcd_init();
    bc_module_lcd_set_font(&bc_font_ubuntu_13);

    bc_led_init_virtual(&led_lcd_red, BC_MODULE_LCD_LED_RED, bc_module_lcd_get_led_driver(), true);
    bc_led_init_virtual(&led_lcd_green, BC_MODULE_LCD_LED_GREEN, bc_module_lcd_get_led_driver(), true);

    // Initialize LoRa
    bc_cmwx1zzabz_init(&lora, BC_UART_UART1);
    bc_cmwx1zzabz_set_event_handler(&lora, lora_callback, NULL);
    bc_cmwx1zzabz_set_class(&lora, BC_CMWX1ZZABZ_CONFIG_CLASS_C);

    // Initialize AT command interface
    at_init(&led, &lora);
    static const bc_atci_command_t commands[] = {
            AT_LORA_COMMANDS,
            AT_LED_COMMANDS,
            BC_ATCI_COMMAND_CLAC,
            BC_ATCI_COMMAND_HELP
    };
    bc_atci_init(commands, BC_ATCI_COMMANDS_LENGTH(commands));

    bc_led_init_virtual(&gps_led_r, BC_MODULE_GPS_LED_RED, bc_module_gps_get_led_driver(), 0);
    bc_led_init_virtual(&gps_led_g, BC_MODULE_GPS_LED_GREEN, bc_module_gps_get_led_driver(), 0);

    bc_servo_init(&servo, BC_PWM_P8);
    bc_servo_set_angle(&servo, 120);

    bc_module_relay_init(&relay_module, 0x3B);
    bc_module_relay_set_state(&relay_module, false);

    servo_task_id = bc_scheduler_register(servo_task_stop, NULL, BC_TICK_INFINITY);

}

void application_task(void)
{
    // Logging in action
    bc_log_debug("application_task run");

    if (!bc_cmwx1zzabz_is_ready(&lora))
    {
        bc_scheduler_plan_current_relative(100);

        return;
    }

    lora_send();

    // Plan next run this function after 1000 ms
    bc_scheduler_plan_current_from_now(5 * 60 * 1000);
}
