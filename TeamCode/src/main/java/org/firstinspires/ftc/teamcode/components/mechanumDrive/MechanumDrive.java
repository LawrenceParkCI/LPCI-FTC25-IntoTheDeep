package org.firstinspires.ftc.teamcode.components.mechanumDrive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * FTC Into The Deep 24-25 <br>
 * A class for using a mechanum drive train,
 * This class does not support dead wheel odometry.
 * <br><br>
 * Last Updated: November 7th, 2024
 * @author Connor Feeney
 */
public class MechanumDrive {
    @FunctionalInterface
    private interface Action{
        void execute(float lY, float lX, float rX);
    }

    private Action driveAction = this::driveRobotCentric;
    private boolean fieldCentric = false;

    private final IMU imu;

    private final DcMotor leftFrontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightFrontMotor;
    private final DcMotor rightBackMotor;

    /**
     * MechanumDrive Constructor, All motors should have a positive forward power.
     * @param leftFrontMotor Your robots left front motor
     * @param leftBackMotor Your robots left back motor
     * @param rightFrontMotor Your robots right front motor
     * @param rightBackMotor Your robots right back motor
     */
    public MechanumDrive(DcMotor leftFrontMotor, DcMotor leftBackMotor, DcMotor rightFrontMotor, DcMotor rightBackMotor, HardwareMap hardwareMap){
        //Store pointer to motor objects
        this.leftFrontMotor = leftFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightBackMotor = rightBackMotor;

        //Get the imu pointer
        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
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
        leftFrontMotor.setPower((-lY + lX + rX) / denominator);
        leftBackMotor.setPower((-lY - lX + rX) / denominator);
        rightFrontMotor.setPower((-lY - lX - rX) / denominator);
        rightBackMotor.setPower((-lY + lX - rX) / denominator);
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
        double y = -lY;
        double rotX = (double) lX * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = (double) lX * Math.sin(-heading) - y * Math.cos(-heading);

        //Calculate denominator
        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(rX), 1);

        //Set all motor powers
        leftFrontMotor.setPower((rotY + rotX + rX) / denominator);
        leftBackMotor.setPower((rotY - rotX + rX) / denominator);
        rightFrontMotor.setPower((rotY - rotX - rX) / denominator);
        rightBackMotor.setPower((rotY + rotX - rX) / denominator);
    }
}
