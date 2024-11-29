package org.firstinspires.ftc.teamcode.components.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.components.util.PID;

public class Intake {
    private final LinearOpMode opMode;

    private final CRServo intakeServo;
    private final CRServo rackServo;
    private final DcMotor intakeMotor;

    private double targetRotation = 0;
    private double maxEncoderValue = 0;

    private boolean extended = false;
    DigitalChannel extendedSensor;
    DigitalChannel retractedSensor;

    PID pid = new PID(0,0,0);
    double kcos = 0;

    public Intake(CRServo intakeServo, CRServo rackServo, DcMotor intakeMotor, DigitalChannel extendedSensor, DigitalChannel retractedSensor, LinearOpMode opMode){
        this.intakeServo = intakeServo;
        this.rackServo = rackServo;
        this.intakeMotor = intakeMotor;

        this.extendedSensor = extendedSensor;
        this.retractedSensor = retractedSensor;

        this.opMode = opMode;
    }

    public void setPidConstant(double kp, double ki, double kd, double a, double maxIntegral) throws IllegalArgumentException{
        pid.setConstants(kp, ki, kd, a, maxIntegral);
    }

    public void setGravityOffset(double kcos){
        this.kcos = kcos;
    }

    public void setMaxEncoderValue(double maxEncoderValue) throws IllegalArgumentException{
        if(maxEncoderValue <= 0){
            throw new IllegalArgumentException("Max Encoder Ticks must be > 0");
        }
        this.maxEncoderValue = maxEncoderValue;
    }

    public void setPosition(double rotation, boolean extended) throws IllegalArgumentException{
        if(rotation > 360 || rotation < 0){
            throw new IllegalArgumentException("rotation must be 0-360");
        }
        targetRotation = (rotation/360);
        this.extended = extended;
    }

    public void runIntake(Boolean run){
        if(run){
            intakeServo.setPower(1);
        }else {
            intakeServo.setPower(0);
        }
    }

    public void reverseIntake(boolean reverse){
        if(reverse){
            intakeServo.setPower(-1);
        }else {
            intakeServo.setPower(0);
        }
    }

    public void update(){
        if(extended && !extendedSensor.getState()){
            rackServo.setPower(1);
        } else if (!extended && !retractedSensor.getState()) {
            rackServo.setPower(-1);
        }else{
            rackServo.setPower(0);
        }

        intakeMotor.setPower(pid.calculate(intakeMotor.getCurrentPosition()/maxEncoderValue, targetRotation) + Math.cos(Math.toRadians(targetRotation * 360)) * kcos);
    }

    public void bufferTelemetry(){
        opMode.telemetry.addData("Intake Angle: ", (intakeMotor.getCurrentPosition()/maxEncoderValue)*360);
        opMode.telemetry.addData("Extended: ", extended);

        opMode.telemetry.addData("Rack Power: ", rackServo.getPower());
        opMode.telemetry.addData("Angle Motor Power: ", intakeMotor.getPower());
    }
}
