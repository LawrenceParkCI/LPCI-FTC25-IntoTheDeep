package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.components.mechanumDrive.MechanumDrive;

@TeleOp(name = "DriveTest Test", group = "Test")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        /*
        TODO: Remember to set all motors to have a forward power

        Example:
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        */

        MechanumDrive d = new MechanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            d.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_y);

            telemetry.addData("Status: ", "Active");
            telemetry.addData("lY: ", -gamepad1.left_stick_y);
            telemetry.addData("lX: ", gamepad1.left_stick_x);
            telemetry.addData("rY: ", gamepad1.right_stick_y);
        }
    }
}
