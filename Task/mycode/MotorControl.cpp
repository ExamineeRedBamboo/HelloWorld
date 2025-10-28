/*
 * @Author: Redbamboo 550362161@qq.com
 * @Date: 2025-10-26 23:30:14
 * @LastEditors: Redbamboo 550362161@qq.com
 * @LastEditTime: 2025-10-28 23:20:34
 * @FilePath: \Task\mycode\MotorControl.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "MotorControl.hpp"

PID::PID(double kp, double ki, double kd, double min, double max)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    min_ = min;
    max_ = max;
}
double PID::compute(double setpoint, double measured, double dt)
{
    double error = setpoint - measured;

    if (error > max_)
    {
        error -= max_ - min_;
    }
    else if (error < min_)
    {
        error += max_ - min_;
    }

    integral += error * dt;

    double derivative = (error - last_error) / dt;
    last_error = error;

    return kp_ * error + ki_ * integral + kd_ * derivative;
}

G6020_Motor::G6020_Motor(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id)
{
    hfdcan_ = hfdcan;
    motor_id_ = motor_id;
}

void G6020_Motor::setCurrent(int current)
{
    uint8_t data[8] = {0};
    data[6] = current & 0xFF;        // 电流低字节
    data[7] = (current >> 8) & 0xFF; // 电流高字节

    FdcanSendMsg(hfdcan_, data, motor_id_, 8);
}

void G6020_Motor::setSpeed(int16_t speed)
{
    double dt = 0.001;  //控制周期
    int current = (int)speedPID.compute(speed, realSpeed, dt);
    setCurrent(current);
}

void G6020_Motor::setAngle(int16_t angle)
{
    double dt = 0.001;  //控制周期
    int current = (int)anglePID.compute(angle, realAngle, dt);
    setCurrent(current);
}

uint16_t tick = 0;

G6020_Motor motor = G6020_Motor(&hfdcan1, 0x1FE);

int16_t targetSpeed = 0;
int16_t targetAngle = 0;

void MainTask(void)
{
    HAL_IWDG_Refresh(&hiwdg1);
    tick++;
    //追正弦波
    // targetSpeed =   
    // motor.setSpeed(targetSpeed);

    //角度
    if (tick < 2000)
    {
        targetAngle = -150;
    }
    else if (tick < 4000)
    {
        targetAngle = 150;
    }
    else if (tick < 6000)
    {
        targetAngle = 0;
    }
    else if (tick < 8000)
    {
        targetAngle = 60;
    }
    else if (tick < 10000)
    {
        targetAngle = 120;
    }
    else if (tick < 12000)
    {
        targetAngle = 0;
    }
    else if (tick < 14000)
    {
        targetAngle = 45;
    }
    else if (tick < 16000)
    {
        targetAngle = -179;
    }
    else
    {
        targetAngle = 0;
    }
    
    motor.setAngle(targetAngle);
}

void CanInit(void)
{
    HAL_FDCAN_Start(&hfdcan1);

    FdcanFilterInit(&hfdcan1, FDCAN_FILTER_TO_RXFIFO0);

    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void MainInit(void)
{
    CanInit();
    HAL_TIM_Base_Start_IT(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        MainTask();
    }
}