package org.firstinspires.ftc.teamcode.components.mechanumDrive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.mechanumDrive.util.FullStateController;
import org.firstinspires.ftc.teamcode.components.mechanumDrive.util.Point;

import java.util.List;

/**
 * FTC Into The Deep 24-25 <br>
 * A class for using a mechanum drive train,
 * This class does not support dead wheel odometry.
 * <br><br>
 * Last Updated: November 21st, 2024
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

    //Velocity
    private double lastTime = 0;
    private double velocityX = 0;
    private double velocityY = 0;

    //Positions
    private final Point prevEncoderSum = new Point(0, 0);
    private final Point prevPosition = new Point(0,0);
    private final Point position = new Point(0,0);

    //Movement constants
    private double tolerance = 0;
    private double kp = 0;
    private double kv = 0;
    private double TPR = 720;
    private double wheelDiam = 0.1;

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

        lastTime = System.currentTimeMillis();
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
        this.TPR = TPR;
    }

    /**
     * Set your robots wheel diameter.
     * @param wheelDiam wheel diameter
     */
    public void setWheelDiam(double wheelDiam){
        this.wheelDiam = wheelDiam;
    }

    /**
     * Update the robots current position and velocity.
     */
    private void updatePositionVelocity(){
        //Get absolute position
        int longitudinalSum = (leftFrontMotor.getCurrentPosition() + leftBackMotor.getCurrentPosition() + rightFrontMotor.getCurrentPosition() + rightBackMotor.getCurrentPosition()) / 4;
        int lateralSum = (leftFrontMotor.getCurrentPosition() - leftBackMotor.getCurrentPosition() - rightFrontMotor.getCurrentPosition() + rightBackMotor.getCurrentPosition()) / 4;

        //Calculate change in absolute position (robot frame)
        int deltaY = (int) (longitudinalSum - prevEncoderSum.getY());
        int deltaX = (int) (lateralSum - prevEncoderSum.getX());

        //Store previous encoderSum
        prevEncoderSum.setY(longitudinalSum);
        prevEncoderSum.setX(lateralSum);

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double deltaWorldY;
        double deltaWorldX;
        double theta;
        if(Math.abs(heading) <= Math.PI/2){
            theta = Math.PI/2 - Math.abs(heading);
        }else{
            theta = Math.abs(heading) - Math.PI/2;
        }
        theta = heading >= 0 ? theta : theta*-1;

        deltaWorldY = (deltaY * Math.sin(theta) + deltaX * Math.sin(theta)) * ((wheelDiam * Math.PI)/TPR);
        deltaWorldX = (deltaY * Math.cos(theta) + deltaX * Math.cos(theta)) * ((wheelDiam * Math.PI)/TPR);

        position.setY(position.getY() + deltaWorldY);
        position.setX(position.getX() + deltaWorldX);

        double deltaPositionY = position.getY() - prevPosition.getY();
        double deltaPositionX = position.getX() - prevPosition.getX();

        double deltaTime = (System.currentTimeMillis() - lastTime) / 1000;

        velocityY = deltaPositionY / deltaTime;
        velocityX = deltaPositionX / deltaTime;
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
        this.updatePositionVelocity();
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
        leftFrontMotor.setPower((lY + lX + rX) / denominator);
        leftBackMotor.setPower((lY - lX + rX) / denominator);
        rightFrontMotor.setPower((lY - lX - rX) / denominator);
        rightBackMotor.setPower((lY + lX - rX) / denominator);
    }

    /**
     * Drive the robot in a field centric motion based on motion vectors.
     * @param lY Y-Component of the translation vector
     * @param lX X-Component of the translation vector
     * @param rX Rotation vector
     */
    private void driveFieldCentric(float lY, float lX, float rX){
        //Get robots current heading
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //Rotate motion vectors
        double rotX = (double) lX * Math.cos(-heading) - lY * Math.sin(-heading);
        double rotY = (double) lX * Math.sin(-heading) + lY * Math.cos(-heading);

        //Calculate denominator
        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rX), 1);

        //Set all motor powers
        leftFrontMotor.setPower((rotY + rotX + rX) / denominator);
        leftBackMotor.setPower((rotY - rotX + rX) / denominator);
        rightFrontMotor.setPower((rotY - rotX - rX) / denominator);
        rightBackMotor.setPower((rotY + rotX - rX) / denominator);
    }

    /**
     * Get the robots velocity in m/s.
     * @return Robot velocity in m/s
     */
    public double getVelocity(){
        return Math.sqrt(Math.pow(velocityX, 2) + Math.pow(velocityY, 2)); //Sum velocity vectors
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
        this.tolerance = tolerance;
    }

    /**
     * Set gain values for autonomous full state controllers.
     * Note: these will most likely be ver very small values
     * @param kp Position gain
     * @param kv Velocity gain
     */
    public void setDriveGain(double kp, double kv){
        this.kp = kp;
        this.kv = kv;
    }

    /**
     * Drive to a target position at a target velocity.
     * @param x Target x position in m
     * @param y Target y position in m
     * @param velocity Target velocity in m/s
     */
    public void driveTo(double x, double y, double velocity){

    }

    /**
     * Flow a generated trajectory at a target velocity.
     * @param path The path for the robot to follow
     * @param velocity Target velocity
     */
    public void followTrajectory(List<Point> path, double velocity){
        for (Point point : path) {
            this.driveTo(point.getX(), point.getY(), velocity);
        }
    }

    /**
     * Add drive data to the telemetry buffer,
     * You still must call telemetry.update().
     */
    public void bufferTelemetry(){
        opMode.telemetry.addData("Left Front Power: ", leftFrontMotor.getPower());
        opMode.telemetry.addData("Left Back Power: ", leftBackMotor.getPower());
        opMode.telemetry.addData("Right Front Power: ", rightFrontMotor.getPower());
        opMode.telemetry.addData("Right Back Power: ", rightBackMotor.getPower());
    }
}
