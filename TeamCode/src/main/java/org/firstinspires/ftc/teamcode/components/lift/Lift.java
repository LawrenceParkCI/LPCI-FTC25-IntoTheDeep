package org.firstinspires.ftc.teamcode.components.lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.lift.util.PID;

public class Lift {
    private final LinearOpMode opMode;
    private final DcMotor armMotor;
    
    private double maxEncoderTicks;
    private double tolerance = 0;

    private double encoderHeight = 0;

    PID pid = new PID(0,0,0);
    double kg = 0;

    public Lift(DcMotor armMotor, double maxEncoderTicks, LinearOpMode opMode) throws IllegalArgumentException{
        this.opMode = opMode;
        this.armMotor = armMotor;
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(maxEncoderTicks <= 0){
            throw new IllegalArgumentException("Max Encoder Ticks must be > 0");
        }
        this.maxEncoderTicks = maxEncoderTicks;
    }

    public void setPidConstant(double kp, double ki, double kd, double a, double maxIntegral) throws IllegalArgumentException{
        pid.setConstants(kp, ki, kd, a, maxIntegral);
    }

    public void setGravityOffset(double kg){
        this.kg = kg;
    }

    public void update(){
        double error = (armMotor.getCurrentPosition()/maxEncoderTicks) - encoderHeight;
        if(Math.abs(error) > Math.abs(tolerance)){
            armMotor.setPower(pid.calculate(armMotor.getCurrentPosition()/maxEncoderTicks, encoderHeight) + kg);
        }
    }

    public void setMaxEncoderTicks(double maxEncoderTicks) throws IllegalArgumentException{
        if(maxEncoderTicks <= 0){
            throw new IllegalArgumentException("Max Encoder Ticks must be > 0");
        }
        this.maxEncoderTicks = maxEncoderTicks;
    }

    public void setTolerance(double tolerance){
        this.tolerance = tolerance;
    }

    public void setHeight(double height) throws IllegalArgumentException{
        if(height > 1 || height < 0){
            throw new IllegalArgumentException("Height Must > 1 and < 0");
        }
        encoderHeight = maxEncoderTicks * height;
    }

    public void bufferTelemetry(){
        opMode.telemetry.addData("Arm Position: ", armMotor.getCurrentPosition()/maxEncoderTicks);
        opMode.telemetry.addData("Arm Speed: ", armMotor.getPower());
    }
}
