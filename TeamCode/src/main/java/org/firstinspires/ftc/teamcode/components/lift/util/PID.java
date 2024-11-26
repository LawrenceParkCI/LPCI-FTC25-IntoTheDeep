package org.firstinspires.ftc.teamcode.components.lift.util;

public class PID {
    double kp, ki, kd;

    double lastError = 0;

    double a;
    double lastFilterEstimate = 0;

    double maxIntegral = 0;
    double integralSum = 0;

    /**
     * {@link PID PID} Constructor
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @throws IllegalArgumentException If kp, ki, or kd is less than 0
     */
    public PID(double kp, double ki, double kd) throws IllegalArgumentException{
        if(kp < 0 || ki < 0 || kd < 0){
            throw  new IllegalArgumentException("PID constants must be greater than 0");
        }

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /**
     * {@link PID PID} Constructor
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param a Low pass filter gain
     * @throws IllegalArgumentException If kp, ki, kd or a is less than 0 or if a is greater than 1
     */
    public PID(double kp, double ki, double kd, double a) throws IllegalArgumentException{
        //Check errors
        if(kp < 0 || ki < 0 || kd < 0){
            throw  new IllegalArgumentException("PID constants must be greater than 0");
        }

        if(a < 0){
            throw new IllegalArgumentException("a value must be greater than 0");
        }

        if(a > 1){
            throw new IllegalArgumentException("a value cannot be greater than 1");
        }

        //Set values
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.a = a;
    }

    public void setMaxIntegral(double maxIntegral) throws IllegalArgumentException{
        if(maxIntegral < 0){
            throw new IllegalArgumentException("maxIntegral must be greater than 0");
        }

        this.maxIntegral = maxIntegral;
    }

    public void setConstants(double kp, double ki, double kd, double a, double maxIntegral) throws IllegalArgumentException{
        //Check errors
        if(kp < 0 || ki < 0 || kd < 0){
            throw  new IllegalArgumentException("PID constants must be greater than 0");
        }
        if(a < 0){
            throw new IllegalArgumentException("a value must be greater than 0");
        }
        if(a > 1){
            throw new IllegalArgumentException("a value cannot be greater than 1");
        }
        if(maxIntegral < 0){
            throw new IllegalArgumentException("maxIntegral must be greater than 0");
        }

        //Set values
        this.maxIntegral = maxIntegral;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.a = a;
    }

    public double calculate(){
        return 0;
    }
}
