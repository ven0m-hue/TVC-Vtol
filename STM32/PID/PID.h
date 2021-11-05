/*
 * MPU9250.h
 *
 *  Created on: 5-Nov-2021
 *      Author: Venom
 */
#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
#include "math.h"

typedef struct 
{
    /* PID Gains*/
    float kp;
    float ki;
    float kd;

    /* P I D terms */
    float propotional;
    float integrator;
    float derivative;

    /*Error term*/
    float prevErr; 

    /*Measurement*/
    float prevMeasure; //Instead of the "error" term, for anti derivative kick


    /*Lowpass Filter Tau for Diffrentiator*/
    float tau;

    /*Sample time */
    float Ts;

    /*Limits, to clamp the windup error and the system saturation*/
    float limMin;
    float limMax;

    float limMinInt;
    float limMaxInt;

    /*And ofcourse, the output*/
    float pidout;

}PID_Handle_t;

//////////////////////////////////////////////////////API_CALLS///////////////////////////////////////
/*Initializer*/
uint8_t PID_Init(PID_Handle_t *pid);
/*PID computation API*/
float PID_Compute(PID_Handle_t *pid, float measurement, float setPoint);


#endif //endo of file