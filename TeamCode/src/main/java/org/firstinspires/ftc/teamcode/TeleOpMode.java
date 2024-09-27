package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.components.DriveTrain;

@TeleOp(name="FTC 2024-2025 TeleOp", group="Linear")
public class TeleOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrain driveTrain = new DriveTrain();
        driveTrain.autoInit(101.6f, 100.0f, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD, this);

        waitForStart();
        while(opModeIsActive()) {
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);

            driveTrain.updateTelemetryEX(telemetry);
            telemetry.update();
        }
    }
}