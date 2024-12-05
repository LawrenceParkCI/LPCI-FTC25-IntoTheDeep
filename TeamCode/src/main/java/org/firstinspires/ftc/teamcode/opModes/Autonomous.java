package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name= "FTC 24-25 Autonomous", group = "Autonomous")
public class Autonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status: ", "INIT");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            leftFrontMotor.setPower(1);
            leftBackMotor.setPower(-1);

            rightFrontMotor.setPower(-1);
            rightBackMotor.setPower(1);

            telemetry.addData("Status: ", "ACTIVE");
            telemetry.update();
        }
    }
}
