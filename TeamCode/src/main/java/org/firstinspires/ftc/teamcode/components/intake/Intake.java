package org.firstinspires.ftc.teamcode.components.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.components.util.PID;

public class Intake {
    private final LinearOpMode opMode;

    private final CRServo intakeServo;
    private final CRServo rackServo;
    private final DcMotor intakeMotor;

    private double targetRotation = 0;
    private double maxEncoderValue = 1;

    private boolean extended = false;
    TouchSensor extendedSensor;
    TouchSensor retractedSensor;

    PID pid = new PID(0,0,0);
    double kcos = 0;

    public Intake(CRServo intakeServo, CRServo rackServo, DcMotor intakeMotor, TouchSensor extendedSensor, TouchSensor retractedSensor, double maxEncoderValue, LinearOpMode opMode){
        this.intakeServo = intakeServo;
        this.rackServo = rackServo;
        this.intakeMotor = intakeMotor;
        this.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.extendedSensor = extendedSensor;
        this.retractedSensor = retractedSensor;

        this.maxEncoderValue = maxEncoderValue;

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

    public void setPosition(double rotation, boolean extended){
        targetRotation = rotation/360;
        this.extended = extended;
    }

    public void runIntake(Boolean run){
        if(run){
            intakeServo.setPower(-1);
        }else {
            intakeServo.setPower(0);
        }
    }

    public void reverseIntake(boolean reverse){
        if(reverse){
            intakeServo.setPower(1);
        }else {
            intakeServo.setPower(0);
        }
    }

    public boolean getExtended(){
        return extended;
    }

    public void update(){
        if(extended && !extendedSensor.isPressed()){
            rackServo.setPower(-1);
        } else if (!extended && !retractedSensor.isPressed()) {
            rackServo.setPower(1);
        }else{
            rackServo.setPower(0);
        }

        double power = -pid.calculate(intakeMotor.getCurrentPosition()/maxEncoderValue, targetRotation);
        opMode.telemetry.addData("Power: ", power);
        intakeMotor.setPower(power);
    }

    public void bufferTelemetry(){
        opMode.telemetry.addData("Extend?: ", extended);
        opMode.telemetry.addData("Intake Pos: ", intakeMotor.getCurrentPosition()/maxEncoderValue);
        opMode.telemetry.addData("Target: ", targetRotation);
        opMode.telemetry.addData("Intake Power: ", intakeMotor.getPower());
    }
}
