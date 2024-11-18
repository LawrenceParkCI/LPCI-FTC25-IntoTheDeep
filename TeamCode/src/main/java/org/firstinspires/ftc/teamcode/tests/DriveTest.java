package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.mechanumDrive.MechanumDrive;

@TeleOp(name = "DriveTest Test", group = "Test")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Get all motor objects
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        //Internally set motor direction (Drive train specific)
        rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Create drive object and set imu orientation
        MechanumDrive d = new MechanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, this);
        d.setImuOrientation(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        Gamepad currGamepad = new Gamepad();
        Gamepad prevGamepad = new Gamepad();

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            //Record gamepad
            prevGamepad.copy(currGamepad);
            currGamepad.copy(gamepad1);

            //Switch driving mode on gamepad a press
            if(currGamepad.a && !prevGamepad.a){
                d.setFieldCentric(!d.getFieldCentric());
            }

            //Drive the robot
            d.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //Update telemetry
            telemetry.addData("Status: ", "Active");
            d.bufferTelemetry();
            telemetry.update();
        }
    }
}
