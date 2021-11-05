/*
 * MPU9250.h
 *
 *  Created on: 5-Nov-2021
 *      Author: Venom
 */

#include "PID.h"

/*
    In the main do the following,

    Set the Kp, Ki, Kd.
    Set the Max/Min lim for antiwind up.
    Set the Max/Min lim for System anti saturation.
*/

/*Initializer*/
uint8_t PID_Init(PID_Handle_t *pid)
{

    /*Clear all the residuals*/

    pid->derivative = 0.0f;
    pid->integrator = 0.0f;
    pid->propotional = 0.0f;

    pid->prevErr = 0.0f;
    pid->prevMeasure = 0.0f;

    pid->pidout = 0.0f;

}

/*PID computation API*/
float PID_Compute(PID_Handle_t *pid, float measurement, float setPoint)
{
    float err = setPoint - measurement;

    /*propotional*/
    pid->propotional = pid->kp * err;

    /*intergral*/
    pid->integrator = 0.5 * pid->Ts * pid->ki * (err + pid->prevErr) + pid->integrator;

    /*
        Clamping logic for integral antiwindup
    */


    /*Diffrentiator*/
    //After accounting the effect of increased BW aka increased High freq response 
    //And the Derivative kick effect.
    pid->derivative = - (pid->kd * (measurement - pid->prevMeasure) + (2*pid->tau + pid->Ts) * pid->derivative)
                        /(2 * pid->tau + pid->Ts);


    //And finally the output 
    
    pid->pidout = pid->propotional + pid->integrator + pid->derivative;
    //Wait wait, where you running at??

    //Acount the system saturation effect 
    if(pid->pidout > pid->limMax)
        pid->pidout = pid->limMax;

    else if(pid->pidout < pid->limMin)
        pid->pidout = pid->limMin;

    //Variable exchange
    pid->prevErr = err;
    pid->prevMeasure = measurement;

    //Okay now!
    return pid->pidout;
}