package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "Motor Test", group = "Test")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, "testMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        telemetry.addData("Status: ", "INIT");
        while(opModeIsActive()){
            motor.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Status: ", "ACTIVE");
            telemetry.addData("Motor Position: ", motor.getCurrentPosition());
            telemetry.addData("Motor Power: ", motor.getPower());
            telemetry.update();
        }
    }
}
