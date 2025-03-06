package org.firstinspires.ftc.teamcode.components.lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.util.PID;

/**
 * FTC Into The Deep 24-25 <br>
 * A class for the LPCI 24-24 Lift
 * <br><br>
 * Last Updated: January 10th, 2025
 * @author Connor Feeney
 */
public class Lift {
    private final LinearOpMode opMode;
    private final DcMotor liftMotor;
    private final DcMotor liftMotorD;
    
    private double maxEncoderValue;
    private double tolerance = 0;

    private double targetHeight = 0;

    PID pid = new PID(0,0,0);
    double kg = 0;

    public Lift(DcMotor liftMotor, DcMotor liftMotorD, double maxEncoderValue, LinearOpMode opMode) throws IllegalArgumentException{
        this.opMode = opMode;

        this.liftMotor = liftMotor;
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.liftMotorD = liftMotorD;
        this.liftMotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(maxEncoderValue <= 0){
            throw new IllegalArgumentException("Max Encoder Ticks must be > 0");
        }
        this.maxEncoderValue = maxEncoderValue;
    }

    public void setPIDConstant(double kp, double ki, double kd, double a, double maxIntegral) throws IllegalArgumentException{
        pid.setConstants(kp, ki, kd, a, maxIntegral);
    }

    public void setGravityOffset(double kg){
        this.kg = kg;
    }

    public double getMaxEncoderValue(){
        return maxEncoderValue;
    }

    public void update(){
        double error = targetHeight - (liftMotor.getCurrentPosition() / maxEncoderValue);
        if(Math.abs(error) > Math.abs(tolerance)){
            double power = pid.calculate(liftMotor.getCurrentPosition() / maxEncoderValue, targetHeight) + kg;
            liftMotor.setPower(power);
            liftMotorD.setPower(power);
        }
    }

    public void setMaxEncoderValue(double maxEncoderValue) throws IllegalArgumentException{
        if(maxEncoderValue <= 0){
            throw new IllegalArgumentException("Max Encoder Ticks must be >= 0");
        }
        this.maxEncoderValue = maxEncoderValue;
    }

    public void setTolerance(double tolerance){
        this.tolerance = tolerance;
    }

    public void setHeight(double height) throws IllegalArgumentException{
        if(height > 1 || height < 0){
            throw new IllegalArgumentException("Height Must <= 1 and >= 0");
        }
        targetHeight = height;
    }

    public void bufferTelemetry(){
        opMode.telemetry.addLine("====Lift Data====");

        opMode.telemetry.addLine("---Motor---");
        opMode.telemetry.addData("Lift Height: ", liftMotor.getCurrentPosition() / maxEncoderValue);
        opMode.telemetry.addData("Target Height: ", targetHeight);
        opMode.telemetry.addData("Lift Motor Power: ", liftMotor.getPower());
        opMode.telemetry.addData("Encoder: ", liftMotor.getCurrentPosition());

        opMode.telemetry.addLine();
    }
}
