package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.intake.Intake;

@TeleOp(name = "Intake Test", group = "Test")
@Config
public class IntakeTest extends LinearOpMode {
    public static double KP = 0;
    public static double KI = 0;
    public static double KD = 0;

    public static double A = 0;
    public static double MAX_INTEGRAL = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wristL = hardwareMap.get(Servo.class, "wristL");
        Servo wristR = hardwareMap.get(Servo.class, "wristR");
        Servo placer = hardwareMap.get(Servo.class, "placer");
        DcMotor controlMotor = hardwareMap.get(DcMotor.class, "controlMotor");

        controlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Gamepad currentGamepad = new Gamepad();
        Gamepad prevGamepad = new Gamepad();

        claw.setPosition(0.5);
        wristL.setPosition(0.5);
        wristR.setPosition(0.5);
        placer.setPosition(0.5);

        int status = 0;

        Intake intake = new Intake(claw, wristL, wristR, placer, controlMotor, 220, this);

        telemetry.addData("Status: ", "INIT");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            prevGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if(currentGamepad.a && !prevGamepad.a){
                status++;
                if(status > 2){
                    status = 0;
                }
            }

            if(currentGamepad.dpad_up & !prevGamepad.dpad_up){
                switch (status){
                    case 0:
                        wristL.setPosition(wristL.getPosition() + 0.01);
                        wristR.setPosition(wristR.getPosition() - 0.01);
                        break;
                    case 1:
                        claw.setPosition(claw.getPosition() + 0.01);
                        break;
                    case 2:
                        placer.setPosition(placer.getPosition() + 0.01);
                        break;
                }
            }

            if(currentGamepad.dpad_down & !prevGamepad.dpad_down){
                switch (status){
                    case 0:
                        wristL.setPosition(wristL.getPosition() - 0.01);
                        wristR.setPosition(wristR.getPosition() + 0.01);
                        break;
                    case 1:
                        claw.setPosition(claw.getPosition() - 0.01);
                        break;
                    case 2:
                        placer.setPosition(placer.getPosition() - 0.01);
                        break;
                }
            }

            if(currentGamepad.a && !prevGamepad.x){
                intake.setExtension(1);
            }

            if(currentGamepad.b && !prevGamepad.b){
                intake.setExtension(0);
            }

            controlMotor.setPower(-gamepad1.left_stick_y);

            intake.setPIDConstants(KP, KI, KD, A, MAX_INTEGRAL);
            intake.update();

            telemetry.addData("Status: ", "ACTIVE");
            telemetry.addData("Wrist Pos (L | R): ", wristL.getPosition() + " | " + wristR.getPosition());
            telemetry.addData("Claw Pos: ", claw.getPosition());
            telemetry.addData("Placer Pos: ", placer.getPosition());
            telemetry.addData("Extension: ", controlMotor.getCurrentPosition());
            telemetry.addData("Control: ", status);
            intake.bufferTelemetry();
            telemetry.update();
        }
    }
}
