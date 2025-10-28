/*
 * @Author: Redbamboo 550362161@qq.com
 * @Date: 2025-10-26 23:30:23
 * @LastEditors: Redbamboo 550362161@qq.com
 * @LastEditTime: 2025-10-29 00:26:18
 * @FilePath: \Task\mycode\CanInit.HPP
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _CanInit_H_
#define _CanInit_H_

#include "HW_fdcan.hpp"
#include "stm32h7xx_hal.h"
#include "iwdg.h"
#include "tim.h"
#include "stdint.h"
#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

void MainInit(void);

#ifdef __cplusplus
}

class PID 
{
private:
    double kp_;
    double ki_;
    double kd_;

    double min_; //解决周期性问题
    double max_;

    double integral = 0.0;
    double last_error = 0.0;
public:
    
    PID(double kp, double ki, double kd, double min, double max);
    double compute(double setpoint, double measured, double dt);
};

class G6020_Motor 
{
private:
    FDCAN_HandleTypeDef* hfdcan_;
    uint16_t motor_id_;
    PID speedPID = PID(0.11, 0.000012, 0.0003, -32768, 32767);
    PID anglePID = PID(0.08, 0.12, 0.005, -180, 180);

    void setCurrent(int current);
public:
    int16_t realAngle;
    int16_t realSpeed;
    G6020_Motor(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id);
    void setSpeed(int16_t speed);
    void setAngle(int16_t angle);
};

extern G6020_Motor motor;
#endif

#endif