package org.firstinspires.ftc.teamcode.components.mechanumDrive;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * FTC Into The Deep 24-25 <br>
 * A class for using a mechanum drive train,
 * This class does not support dead wheel odometry
 * <br><br>
 * Last Updated: November 7th, 2024
 * @author Connor Feeney
 */
public class MechanumDrive {
    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;

    /**
     * MechanumDrive Constructor, All motors should have a positive forward power
     * @param leftFrontMotor Your robots left front motor
     * @param leftBackMotor Your robots left back motor
     * @param rightFrontMotor Your robots right front motor
     * @param rightBackMotor Your robots right back motor
     */
    public MechanumDrive(DcMotor leftFrontMotor, DcMotor leftBackMotor, DcMotor rightFrontMotor, DcMotor rightBackMotor){
        //Store pointer to motor objects
        this.leftFrontMotor = leftFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightBackMotor = rightBackMotor;
    }

    /**
     * Drive the robot with according values
     * @param lY Left stick y power
     * @param lX Left stick x power
     * @param rY Right stick y power
     */
    public void drive(float lY, float lX, float rY){
        //Set all motor powers
        leftFrontMotor.setPower(-lY + lX + rY);
        leftBackMotor.setPower(-lY - lX + rY);
        rightFrontMotor.setPower(-lY - lX - rY);
        rightBackMotor.setPower(-lY + lX - rY);
    }
}
