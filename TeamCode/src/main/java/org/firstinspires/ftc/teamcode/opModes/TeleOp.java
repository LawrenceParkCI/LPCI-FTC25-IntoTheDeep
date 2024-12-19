package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.components.intake.Intake;
import org.firstinspires.ftc.teamcode.components.lift.Lift;
import org.firstinspires.ftc.teamcode.components.mechanumDrive.MechanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "FTC 24-25 TeleOp", group = "TeleOp")

@Disabled
public class TeleOp extends LinearOpMode {
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

        MechanumDrive d = new MechanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, this);
        d.setImuOrientation(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        d.setFieldCentric(true);

        DcMotor armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        DcMotor dumbMotor = hardwareMap.get(DcMotor.class, "dumbMotor");

        Lift lift = new Lift(armMotor, dumbMotor,7000,this);
        lift.setPidConstant(6.5, 20, 0, 0,0);

        CRServo rackServo = hardwareMap.get(CRServo.class, "rackServo");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        TouchSensor extendedSensor = hardwareMap.get(TouchSensor.class, "extendedSensor");
        TouchSensor retractedSensor = hardwareMap.get(TouchSensor.class, "retractedSensor");

        Intake intake = new Intake(intakeServo, rackServo, intakeMotor, extendedSensor, retractedSensor, 1440, this);

        Gamepad currGamepad = new Gamepad();
        Gamepad prevGamepad = new Gamepad();

        telemetry.addData("Status: ", "INIT");
        telemetry.update();

        waitForStart();

        int intakeState = 0;
        int slideState = 0;

        while (opModeIsActive()){
            prevGamepad.copy(currGamepad);
            currGamepad.copy(gamepad1);

            if(currGamepad.b && !prevGamepad.b){
                intakeState++;
                if(intakeState > 3){
                    intakeState = 0;
                }
            }

            switch (intakeState){
                case 0:
                case 2:
                    intake.runIntake(false);
                    break;
                case 1:
                    intake.runIntake(true);
                    break;
                case 3:
                    intake.reverseIntake(true);
                    break;
            }

            if(currGamepad.a && !prevGamepad.a){
                slideState++;
                if(slideState > 1){
                    slideState = 0;
                }
            }

            switch(slideState){
                case 0:
                    lift.setHeight(0);
                    break;
                case 1:
                    lift.setHeight(0.9);
                    break;
            }

            if(currGamepad.x && !prevGamepad.x){
                intake.setPosition(0, !intake.getExtended());
            }

            if(currGamepad.left_bumper){
                intakeMotor.setPower(0.4);
            } else if(currGamepad.right_bumper){
                intakeMotor.setPower(-0.4);
            } else{
                intakeMotor.setPower(0);
            }

            d.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            lift.update();
            intake.update();

            telemetry.addData("Status: ", "ACTIVE");
            telemetry.update();
        }
    }

}
