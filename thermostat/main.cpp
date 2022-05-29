#include "mbed.h"
#include "board_freedom.h"
#include "fsl_adc16.h"
#include <cstdio>
#include <cstdlib>
#include "rtos.h"
#include "adc.h"
#include "oled_ssd1322.h"
#include <list>
#include <tuple>
#include <mbed_error.h>

float get_temp(uint16_t analog_in);

enum class Trend {
    INCREASING,
    STABLE,
    DECREASING,
};

const uint16_t min_temp = 21517;
const uint16_t max_temp = 23648;
const float upper_temp = 35;
const float lower_temp = 30;
const float max_error = 0.5;
const float compensated_max_temp = upper_temp - max_error;
const float compensated_min_temp = lower_temp + max_error;

const uint32_t MAX_MSG_SIZE = 30;
char MSG[MAX_MSG_SIZE];
const char FORMAT[] = "Raw %i, Temp is: %f\t";

enum class SensorStatus {
    PRESENT,
    ABSENT,
};

SensorStatus sensor_status = SensorStatus::PRESENT;

const uint32_t NEW_TEMP = 0b0001;
const uint32_t DISPLAY_TEMP = 0b0010;
EventFlags control_flags;
uint32_t last_temp = 0;
PwmOut heater(PTC2);
DigitalOut redLed(PTB2);
DigitalOut greenLed(LED_GREEN);

void execute_adc_conversion() {
    last_temp = adc_read(ADC0_SE16);
    control_flags.set(NEW_TEMP);
}

enum class RangeEnum {
    OVER,
    INSIDE,
    UNDER
};

struct SystemStatus {
    float prev_temp;
    SensorStatus sensorStatus;
};

RangeEnum in_range(float value, float min, float max){
    if(value < min) return RangeEnum::UNDER;
    else if(value > max) return RangeEnum::OVER;
    return RangeEnum::INSIDE;
}

std::tuple<RangeEnum, bool> get_status(SystemStatus &status, float new_temp) {
    auto is_error = false;
    float diff = abs(new_temp - status.prev_temp);
    if(diff > 5 && status.sensorStatus == SensorStatus::PRESENT){
           status.prev_temp = new_temp;
           status.sensorStatus = SensorStatus::ABSENT;
        is_error = true;
    } else {
        status.sensorStatus = diff < 5 ? SensorStatus::PRESENT : SensorStatus::ABSENT;
    }
    auto range = in_range(new_temp, compensated_min_temp, compensated_max_temp);
    return std::make_tuple(range, is_error);
}

SystemStatus sys_status;
void heater_control() {
    while(true){
        uint32_t result = control_flags.wait_any(NEW_TEMP, true);
        if((result & NEW_TEMP) == 0) continue;
        float new_temp_f = get_temp(last_temp);
        auto status = get_status(sys_status, new_temp_f);
        auto is_error = std::get<1>(status);
        if(is_error) {
            redLed = false;
            continue;
        }
        auto range_status = std::get<0>(status);
        greenLed = !(range_status == RangeEnum::INSIDE);
        auto heater_status = range_status != RangeEnum::OVER;
        printf("Range status %i - temp: %f\n", range_status, new_temp_f);
        redLed = heater_status;
        heater = heater_status;
        control_flags.set(DISPLAY_TEMP);
    }
}

void lcd_display_control() {
    while(true) {
        uint32_t flags_read = control_flags.wait_any(DISPLAY_TEMP, true);
        if((flags_read & DISPLAY_TEMP) == 0)
            continue;
        if(sys_status.sensorStatus == SensorStatus::ABSENT) {
            sprintf(MSG, "Error, no sensor");
        } else {
            uint32_t temp_read = last_temp;
            sprintf(MSG, FORMAT, temp_read, get_temp(temp_read));
        }

        u8g2_ClearBuffer(&oled);
        u8g2_DrawUTF8(&oled, 10, 10, MSG);
        u8g2_SendBuffer(&oled);
    }
}

void handle_error(const mbed_error_ctx* error_ctx) {
    heater = false;
}

const char lcd_name[] = "lcd_display_thread";
const char heater_name[] = "heater_thread";
// main() runs in its own thread in the OS
int main()
{
    board_init();
    u8g2_ClearBuffer(&oled);
    u8g2_SetFont(&oled, u8g2_font_6x12_mr);
    u8g2_SendBuffer(&oled);
    
    Thread *lcd_display_thread = 
        new Thread(osPriorityNormal, OS_STACK_SIZE, nullptr, lcd_name);
    Thread *heater_thread =
        new Thread(osPriorityNormal, OS_STACK_SIZE, nullptr, heater_name);
    
    uint16_t first_temp = adc_read(ADC0_SE16);
    float first_temp_f = get_temp(first_temp);
    printf("First temp %f", first_temp_f);
    sys_status.prev_temp = first_temp_f;
    sys_status.sensorStatus = first_temp_f < -2 ? 
                            SensorStatus::ABSENT : 
                            SensorStatus::PRESENT;

    lcd_display_thread->start(lcd_display_control);
    heater_thread->start(heater_control);

    mbed_set_error_hook(handle_error);
    LowPowerTicker ticker;
    ticker.attach(&execute_adc_conversion, 100ms);
    
    while (true) {
        sleep_manager_sleep_auto();
    }
}

float get_temp(uint16_t analog_in) {
    float volts = analog_in * 3 / 65535.0;
    return (volts * 1000 - 400) / 19.5f;
}