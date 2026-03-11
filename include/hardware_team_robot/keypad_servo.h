#pragma once

#include <pigpio.h>
#include <map>
#include <thread>
#include <chrono>

class KeypadServo
{

public:

    KeypadServo(int servo_x_pin, int servo_y_pin)
    {
        servo_x_ = servo_x_pin;
        servo_y_ = servo_y_pin;
    }

    bool initialize()
    {

        key_map_ =
        {
            {'1',{1200,1700}},
            {'2',{1500,1700}},
            {'3',{1800,1700}},

            {'4',{1200,1500}},
            {'5',{1500,1500}},
            {'6',{1800,1500}},

            {'7',{1200,1300}},
            {'8',{1500,1300}},
            {'9',{1800,1300}},

            {'0',{1500,1100}},
            {'#',{1800,1100}}
        };

        return home();
    }

    void press_digit(char digit)
    {

        auto it = key_map_.find(digit);

        if(it == key_map_.end())
            return;

        int x = it->second.first;
        int y = it->second.second;

        move_to(x,y);
        press();
    }

    bool KeypadServo::home()
{
    int ret_x = gpioServo(servo_x_, 1500);
    int ret_y = gpioServo(servo_y_, 1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    return (ret_x >= 0 && ret_y >= 0);

    }

private:

    int servo_x_;
    int servo_y_;

    int press_depth_ = 250;

    std::map<char,std::pair<int,int>> key_map_;

    void move_to(int x_pwm,int y_pwm)
    {

        gpioServo(servo_x_,x_pwm);
        gpioServo(servo_y_,y_pwm);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void press()
    {

        int current = gpioGetServoPulsewidth(servo_y_);

        gpioServo(servo_y_, current + press_depth_);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        gpioServo(servo_y_, current);

        std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }

};
