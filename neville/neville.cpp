#include "mbed.h"
#include "physcom.h"
#include <assert.h>
#include <iostream>

enum MotorLabel {LeftMotor, RightMotor};

//Completely covered: ~0.18
//
//with flashlight on top: ~0.003

const uint32_t black_threshold = 750; // 0 -> 750 is not black
const float ambient_light_threshold = 0.08;
const uint32_t range_threshold = 10;
const uint8_t SENSOR_COUNT = 5;
const float ref_speed = 0.35;
//const float refresh_rate = 0.001;         // seconds

physcom::M3pi robot; //create an object of type  M3pi
Serial pc(USBTX, USBRX); // open a serial communication to terminal emulator
AnalogIn passive_light(p20);

void set_motor(MotorLabel motor, float speed){
    robot.activate_motor(motor, speed);
}

// -1 <= orientation <= +1
void move_rotate(float orientation, float def_speed){         
    float left_speed = def_speed, right_speed = def_speed, rotation_ratio;
    if (orientation < 0){
        rotation_ratio = -2 * orientation;
        right_speed += def_speed * rotation_ratio;
        left_speed -= def_speed * rotation_ratio;
    }
    else {
        rotation_ratio = 2 * orientation;
        right_speed -= def_speed * rotation_ratio;
        left_speed += def_speed * rotation_ratio;
    }
    
    set_motor(LeftMotor, left_speed);
    set_motor(RightMotor, right_speed);
}

float weighted_avg(int* sensors, int sensor_count){
    assert(sensor_count == 5);
    float center_white_ratio = 1 - sensors[2] / 1000;
    float center_white_ratio_adjusted = (center_white_ratio + 0.2) / 1.2;
    
    return ((- 2.2 *sensors[0] - 1.2*sensors[1] + 1.2*sensors[3] + 2.2*sensors[4]) * center_white_ratio_adjusted) / 3400.0f;
}

bool is_infront_obstacle(uint32_t range){
    return range < range_threshold;
}

bool is_in_tunnel(float ambient_light){
    return ambient_light > ambient_light_threshold;
}

float get_speed(int *sensors, int sensor_count){
    assert(sensor_count == 5);
    static bool paused = false;
    bool sensors_black[5];
    uint8_t black_sensors = 0;
    for (uint32_t i = 0; i < 5; i++){
        sensors_black[i] = sensors[i] > black_threshold;
        if (sensors_black[i])
            black_sensors++;
    }
    switch (black_sensors){
        case 0: {
            paused = false;
            return ref_speed / 1.2;
        }
        
        case 1: {
            paused = false;            
            return sensors_black[2] ? ref_speed * 1.1 : ref_speed / 1.2; 
        }
        
        case 2:
        case 4: {
            paused = false;            
            return ref_speed / 1.2;    
        }
        
        case 3: {
            paused = false;
            if (sensors_black[1] && sensors_black[2] && sensors_black[3])
                return ref_speed;
            return ref_speed / 1.2;
        }
        
        case 5: {
            move_rotate(0, 0);
            exit(1);
        }
        
        default:
            assert(false);
    }
    return 0.0;
}

int main() {
    int sensors[SENSOR_COUNT];
    
    AnalogIn passive_light(p20);
    DigitalOut redled(p12);
    DigitalOut greenled(p19);
    redled = greenled = 0;
    physcom::Ping pinger(p11);
    
    pc.printf("Start calibration!\r\n");
    robot.sensor_auto_calibrate();   //robot executes calibration
    pc.printf("Finished calibration!\r\n");
    
    uint32_t iteration = 0;

    while (true) {
        robot.calibrated_sensors(sensors);
        //pc.printf("sensor values: %d; %d; %d; %d; %d;\r\n", sensors[0], sensors[1], sensors[2], sensors[3], sensors[4]);
        float ambient_light = passive_light.read();
        float orientation = weighted_avg(sensors, SENSOR_COUNT);
        float speed = get_speed(sensors, SENSOR_COUNT);
                
        redled = greenled = is_in_tunnel(ambient_light);

        if (iteration++ % 500 == 0){
            pinger.Send();
            uint32_t free_range = pinger.Read_cm();
            if (is_infront_obstacle(free_range)){
                orientation = 0.0;
                speed = 0.0;
                iteration = 0;
            }
        }
        move_rotate(orientation, speed);
        //wait(refresh_rate);
    }
}
