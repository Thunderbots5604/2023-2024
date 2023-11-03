package org.firstinspires.ftc.teamcode.math;

public class PIDF {

    final double F = 0.2; //Feed forward
    final double P = 0.1; //Proportional
    final double I = 0.001; //Integral
    final double D = 0.01; //Derivative

    double totalError = 0;
    double lastError = 0;


    /*
    F = Feed Forward
        Gives estimate of distance traveled
    Motor power = F * speed
    Find F by finding motor power and speed.
    Speed = Distance / Time

    P = Proportional
        motor power = F * speed  +  P * error
    Use sensor to get information
    Error = Wanted speed - Actual speed
    motor power = F * wanted speed  +  P * (wanted speed - actual speed)
    How to find error : Test different P values to find one that works

     */
    public double motorPower(double speed, double measuredSpeed){
        double error = speed - measuredSpeed;
        totalError += error;
        double deltaError = error - lastError;
        lastError = error;
        return F * speed + P * error + I * totalError + D * deltaError;
    }




}
