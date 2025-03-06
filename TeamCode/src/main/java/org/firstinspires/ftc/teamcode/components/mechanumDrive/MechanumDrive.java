package org.firstinspires.ftc.teamcode.components.mechanumDrive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.mechanumDrive.util.Point;

import java.util.List;

/**
 * FTC Into The Deep 24-25 <br>
 * A class for using a mechanum drive train,
 * This class does not support dead wheel odometry.
 * <br><br>
 * Last Updated: January 10th, 2025
 * @author Connor Feeney
 */
public class MechanumDrive {
    @FunctionalInterface
    private interface Action{
        void execute(float lY, float lX, float rX);
    }

    //Drive mode
    private Action driveAction = this::driveRobotCentric;
    private boolean fieldCentric = false;

    //Control Hub Internals
    private final LinearOpMode opMode;
    private final IMU imu;

    //Motors
    private final DcMotor leftFrontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightFrontMotor;
    private final DcMotor rightBackMotor;

    //Positions
    private int prevEncoderLF = 0;
    private int prevEncoderLB = 0;
    private int prevEncoderRF = 0;
    private int prevEncoderRB = 0;

    private final Point position = new Point(0,0);

    private double wheelDiam = 0.1;

    private double speedModifier = 1.0;

    /**
     * MechanumDrive Constructor, All motors should have a positive forward power.
     * @param leftFrontMotor Your robots left front motor
     * @param leftBackMotor Your robots left back motor
     * @param rightFrontMotor Your robots right front motor
     * @param rightBackMotor Your robots right back motor
     */
    public MechanumDrive(DcMotor leftFrontMotor, DcMotor leftBackMotor, DcMotor rightFrontMotor, DcMotor rightBackMotor, LinearOpMode opMode){
        //Store pointer to motor objects
        this.leftFrontMotor = leftFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightBackMotor = rightBackMotor;

        //Set up motor encoders
        enableEncoder(this.leftFrontMotor);
        enableEncoder(this.leftBackMotor);
        enableEncoder(this.rightFrontMotor);
        enableEncoder(this.rightBackMotor);

        this.opMode = opMode;

        //Get the imu pointer
        imu = this.opMode.hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
    }

    /**
     * Set up motors encoder.
     * @param motor Motor object
     */
    private void enableEncoder(DcMotor motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Set the robots imu / rev hub orientation, this should be set before using field centric movement.
     * This method will also reset the yaw of your imu for consistency.
     * @param logoFacingDirection The direction the logo is facing
     * @param usbFacingDirection The direction the usb port is facing
     * @see RevHubOrientationOnRobot
     * @see RevHubOrientationOnRobot.LogoFacingDirection
     * @see RevHubOrientationOnRobot.UsbFacingDirection
     */
    public void setImuOrientation(RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection, RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection){
        //Create an orientation based on params
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        //Set orientation
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
    }

    /**
     * Set the amount of encoder ticks per rotation of a wheel.
     * @param TPR ticks per rotation
     */
    public void setTPR(double TPR){

    }

    /**
     * Set your robots wheel diameter.
     * @param wheelDiam wheel diameter
     */
    public void setWheelDiam(double wheelDiam){

    }

    /**
     * Update the robots current position.
     */
    private void updatePosition(){
        double deltaLF = (leftFrontMotor.getCurrentPosition() - prevEncoderLF);
        double deltaLB = (leftBackMotor.getCurrentPosition() - prevEncoderLB);
        double deltaRF = (rightFrontMotor.getCurrentPosition() - prevEncoderRF);
        double deltaRB = (rightBackMotor.getCurrentPosition() - prevEncoderRB);

        prevEncoderLF = leftFrontMotor.getCurrentPosition();
        prevEncoderLB = leftBackMotor.getCurrentPosition();
        prevEncoderRF = rightBackMotor.getCurrentPosition();
        prevEncoderRB = rightBackMotor.getCurrentPosition();

        double deltaX = (deltaLF - deltaLB - deltaRF + deltaRB) / 4;
        double deltaY = (deltaLF + deltaLB + deltaRF + deltaRB) / 4;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double deltaWorldX = deltaX * Math.cos(-heading) - deltaY * Math.sin(-heading);
        double deltaWorldY = deltaX * Math.sin(-heading) + deltaY * Math.cos(-heading);

        position.setX(position.getX() + (deltaWorldX  * (wheelDiam * Math.PI)));
        position.setY(position.getY() + (deltaWorldY  * (wheelDiam * Math.PI)));
    }

    /**
     * Set if the mechanum drive should follow a field centric movement style or not
     * Some drivers may prefer filed centric movement as it can make several evasive
     * maneuvers easier while still translating the robot in a given direction.
     * By default the mechanum drive moves in a robot centric movement style.
     * Make sure to properly set the rev hub orientation with {@link #setImuOrientation(RevHubOrientationOnRobot.LogoFacingDirection, RevHubOrientationOnRobot.UsbFacingDirection) setImuOrientation(logoFacingDirection, usbFacingDirection)}
     * @param fieldCentric True to make the robot drive field centric, False to make it robot centric
     * @see #setImuOrientation(RevHubOrientationOnRobot.LogoFacingDirection, RevHubOrientationOnRobot.UsbFacingDirection) setImuOrientation(logoFacingDirection, usbFacingDirection)
     */
    public void setFieldCentric(boolean fieldCentric){
        driveAction = fieldCentric ? this::driveFieldCentric : this::driveRobotCentric;
        this.fieldCentric = fieldCentric;
    }

    /**
     * Check if the robot is set to follow a field centric movement style.
     * @return True if the robot drives field centric
     */
    public boolean getFieldCentric(){
        return fieldCentric;
    }

    /**
     * Drive the robot with according values.
     * @param lY Left stick y power
     * @param lX Left stick x power
     * @param rX Right stick y power
     */
    public void drive(float lY, float lX, float rX){
        driveAction.execute(lY, lX, rX); //Calls to whatever action is set based on setFieldCentric(Boolean fieldCentric)
        this.updatePosition();
    }

    public void setSpeedModifier(double mod){
        this.speedModifier = mod;
    }

    /**
     * Drive the robot in a robot centric motion based on motion vectors.
     * @param lY Y-Component of the translation vector
     * @param lX X-Component of the translation vector
     * @param rX Rotation vector
     */
    private void driveRobotCentric(float lY, float lX, float rX){
        //Calculate denominator
        double denominator = Math.max(Math.abs(lY) + Math.abs(lX) + Math.abs(rX), 1);

        //Set all motor powers
        leftFrontMotor.setPower(((lY + lX + rX) / denominator) * speedModifier);
        leftBackMotor.setPower(((lY - lX + rX) / denominator) * speedModifier);
        rightFrontMotor.setPower(((lY - lX - rX) / denominator) * speedModifier);
        rightBackMotor.setPower(((lY + lX - rX) / denominator) * speedModifier);
    }

    /**
     * Drive the robot in a field centric motion based on motion vectors.
     * @param lY Y-Component of the translation vector
     * @param lX X-Component of the translation vector
     * @param rX Rotation vector
     */
    private void driveFieldCentric(float lY, float lX, float rX) {
        //Get robots current heading
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //Rotate motion vectors
        double rotX = (double) lX * Math.cos(-heading) - lY * Math.sin(-heading);
        double rotY = (double) lX * Math.sin(-heading) + lY * Math.cos(-heading);

        //Calculate denominator
        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rX), 1);

        //Set all motor powers
        leftFrontMotor.setPower(((rotY + rotX + rX) / denominator) * speedModifier);
        leftBackMotor.setPower(((rotY - rotX + rX) / denominator) * speedModifier);
        rightFrontMotor.setPower(((rotY - rotX - rX) / denominator) * speedModifier);
        rightBackMotor.setPower(((rotY + rotX - rX) / denominator) * speedModifier);
    }

    /**
     * Get the robots current position in m.
     * @return Robot position in m
     */
    public Point getPosition(){
        return position.clone(); //Return a clone to avoid direct manipulation
    }

    /**
     * Set the tolerance or maximum amount of error in autonomous driving.
     * @param tolerance your max error
     */
    public void setTolerance(double tolerance){

    }

    /**
     * Drive to a target position at a target velocity.
     * @param x Target x position in m
     * @param y Target y position in m
     */
    public void driveTo(double x, double y){

    }

    /**
     * Flow a generated trajectory at a target velocity.
     * @param path The path for the robot to follow
     */
    public void followTrajectory(List<Point> path){
        for (Point point : path) {
            this.driveTo(point.getX(), point.getY());
        }
    }

    /**
     * Add drive data to the telemetry buffer,
     * You still must call telemetry.update().
     */
    public void bufferTelemetry(){
        opMode.telemetry.addLine("====Drive Data====");

        opMode.telemetry.addLine("---Motor---");
        opMode.telemetry.addData("Left Front Power: ", leftFrontMotor.getPower());
        opMode.telemetry.addData("Left Back Power: ", leftBackMotor.getPower());
        opMode.telemetry.addData("Right Front Power: ", rightFrontMotor.getPower());
        opMode.telemetry.addData("Right Back Power: ", rightBackMotor.getPower());

        opMode.telemetry.addLine("---Position---");
        opMode.telemetry.addData("Heading: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        opMode.telemetry.addData("Position (X, Y): ", position.toString());

        opMode.telemetry.addLine("---Settings---");
        opMode.telemetry.addData("Drive Mode", fieldCentric? "Field Centric" : "Robot Centric");

        opMode.telemetry.addLine();
    }
}
