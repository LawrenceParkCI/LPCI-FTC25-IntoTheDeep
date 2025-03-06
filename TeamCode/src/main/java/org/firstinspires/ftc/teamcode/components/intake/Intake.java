package org.firstinspires.ftc.teamcode.components.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.util.PID;

/**
 * FTC Into The Deep 24-25 <br>
 * A class for the LPCI 24-24 Intake
 * <br><br>
 * Last Updated: January 10th, 2025
 * @author Connor Feeney
 */
public class Intake {
    private final Servo claw;
    private final Servo wristL;
    private final Servo wristR;
    private final Servo placer;
    private final DcMotor controlMotor;
    private final LinearOpMode opMode;

    private double maxEncoderValue;
    private double targetExtension = 0;
    private double tolerance = 0;


    PID pid = new PID(0, 0, 0);

    public Intake(Servo claw, Servo wristL, Servo wristR, Servo placer, DcMotor controlMotor, double maxEncoderValue, LinearOpMode opMode) {
        this.claw = claw;
        this.wristL = wristL;
        this.wristR = wristR;
        this.placer = placer;
        this.controlMotor = controlMotor;
        this.controlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.controlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.opMode = opMode;

        this.setMaxEncoderValue(maxEncoderValue);
    }

    public void setPIDConstants(double kp ,double ki, double kd, double a, double maxIntegral) {
        pid.setConstants(kp, ki, kd, a, maxIntegral);
    }

    public void setMaxEncoderValue(double maxEncoderValue) throws IllegalArgumentException {
        if(maxEncoderValue <= 0 ){
            throw new IllegalArgumentException("Max encoder value must be > 0");
        }

        this.maxEncoderValue = maxEncoderValue;
    }

    public double getMaxEncoderValue() {
        return maxEncoderValue;
    }

    public void setTolerance(double tolerance) throws IllegalArgumentException{
        if(tolerance < 0){
            throw new IllegalArgumentException("Tolerance must be > 0");
        }

        this.tolerance = tolerance;
    }

    public void openClaw() {
        claw.setPosition(0.5);
    }

    public void closeClaw() {
        claw.setPosition(0.23);
    }

    public void wristUp() {
        wristL.setPosition(0.82);
        wristR.setPosition(0.18);
    }

    public void wristDown() {
        wristL.setPosition(0.18);
        wristR.setPosition(0.82);
    }

    public void wristMid() {
        wristL.setPosition(0.5);
        wristR.setPosition(0.5);
    }

    public void openPlacer() {
        placer.setPosition(0.36);
    }

    public void closePlacer() {
        placer.setPosition(0.52);
    }

    public void setExtension(double extension) throws IllegalArgumentException {
        if(extension > 1 || extension < 0){
            throw new IllegalArgumentException("Extension must be in range 0-1");
        }

        targetExtension = extension;
    }

    public void update() {
        double error = targetExtension - (controlMotor.getCurrentPosition() / maxEncoderValue);
        if(Math.abs(error) > tolerance){
            double power = pid.calculate(controlMotor.getCurrentPosition() / maxEncoderValue, targetExtension);
            controlMotor.setPower(power);
        }
    }

    public void bufferTelemetry(){
        opMode.telemetry.addLine("====Intake Data=====");

        opMode.telemetry.addLine("---Motor---");
        opMode.telemetry.addData("Extension: ", controlMotor.getCurrentPosition() / maxEncoderValue);
        opMode.telemetry.addData("Control Motor Power: ", controlMotor.getPower());

        opMode.telemetry.addLine("---Servos---");
        opMode.telemetry.addData("Claw: ", claw.getPosition() == 0.5 ? "Open" : "Closed");
        opMode.telemetry.addData("Wrist: ", wristL.getPosition() == 0.5 ? "Up" : "Down");
        opMode.telemetry.addData("Placer: ", placer.getPosition() == 0.5 ? "Open" : "Closed");

        opMode.telemetry.addLine();
    }
}
