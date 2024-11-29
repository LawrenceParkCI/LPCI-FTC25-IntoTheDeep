package org.firstinspires.ftc.teamcode.components.lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.util.PID;

public class Lift {
    private final LinearOpMode opMode;
    private final DcMotor armMotor;
    
    private double maxEncoderValue;
    private double tolerance = 0;

    private double targetHeight = 0;

    PID pid = new PID(0,0,0);
    double kg = 0;

    public Lift(DcMotor armMotor, double maxEncoderValue, LinearOpMode opMode) throws IllegalArgumentException{
        this.opMode = opMode;
        this.armMotor = armMotor;
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(maxEncoderValue <= 0){
            throw new IllegalArgumentException("Max Encoder Ticks must be > 0");
        }
        this.maxEncoderValue = maxEncoderValue;
    }

    public void setPidConstant(double kp, double ki, double kd, double a, double maxIntegral) throws IllegalArgumentException{
        pid.setConstants(kp, ki, kd, a, maxIntegral);
    }

    public void setGravityOffset(double kg){
        this.kg = kg;
    }

    public void update(){
        double error = (armMotor.getCurrentPosition()/ maxEncoderValue) - targetHeight;
        if(Math.abs(error) > Math.abs(tolerance)){
            armMotor.setPower(pid.calculate(armMotor.getCurrentPosition() / maxEncoderValue, targetHeight) + kg);
        }
    }

    public void setMaxEncoderValue(double maxEncoderValue) throws IllegalArgumentException{
        if(maxEncoderValue <= 0){
            throw new IllegalArgumentException("Max Encoder Ticks must be > 0");
        }
        this.maxEncoderValue = maxEncoderValue;
    }

    public void setTolerance(double tolerance){
        this.tolerance = tolerance;
    }

    public void setHeight(double height) throws IllegalArgumentException{
        if(height > 1 || height < 0){
            throw new IllegalArgumentException("Height Must > 1 and < 0");
        }
        targetHeight = maxEncoderValue * height;
    }

    public void bufferTelemetry(){
        opMode.telemetry.addData("Arm Position: ", armMotor.getCurrentPosition() / maxEncoderValue);
        opMode.telemetry.addData("Arm Speed: ", armMotor.getPower());
    }
}
