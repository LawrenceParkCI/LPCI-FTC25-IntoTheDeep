package org.firstinspires.ftc.teamcode.driveModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.intake.Intake;
import org.firstinspires.ftc.teamcode.components.lift.Lift;
import org.firstinspires.ftc.teamcode.components.mechanumDrive.MechanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name= "FTC 24-25 TeleOp", group = "TeleOp")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status: ", "Setting Up...");
        telemetry.addLine("Do Not Press Start");
        telemetry.update();

        //====Hardware Components====
        //Drive Motors
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Lift Components
        DcMotor liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        DcMotor liftMotorD = hardwareMap.get(DcMotor.class, "liftMotorD");

        //Intake Components
        DcMotor controlMotor = hardwareMap.get(DcMotor.class, "controlMotor");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wristL = hardwareMap.get(Servo.class, "wristL");
        Servo wristR = hardwareMap.get(Servo.class, "wristR");
        Servo placer = hardwareMap.get(Servo.class, "placer");

        //====Robot Components====
        MechanumDrive robot = new MechanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor, this);
        Lift lift = new Lift(liftMotor, liftMotorD, 7000, this);
        Intake intake = new Intake(claw, wristL, wristR, placer, controlMotor, 220 ,this);

        //====Robot Component Configuration====
        robot.setFieldCentric(true);
        lift.setPIDConstant(76.176, 901.704, 3.504, 0, 0);
        intake.setPIDConstants(1.333, 8.228, 0.117,0, 0);

        //GamePads
        Gamepad currGamepad = new Gamepad();
        Gamepad prevGamepad = new Gamepad();

        telemetry.addData("Status: ", "Initialized");
        telemetry.addLine("Press Start");
        telemetry.update();
        waitForStart();

        //States
        int intakeState = 0;
        int liftState = 0;
        int clawState = 0;

        intake.closePlacer();
        intake.openClaw();

        while (opModeIsActive()){
            //Record gamepad for rising and falling edge detectors
            prevGamepad.copy(currGamepad);
            currGamepad.copy(gamepad1);

            //Handle Drive Actions
            robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            if(gamepad1.right_trigger > 0){
                double speedModifier = (2 - gamepad1.right_trigger) * 0.5;
                robot.setSpeedModifier(speedModifier);
            } else {
                robot.setSpeedModifier(1);
            }

            //Handle Lift Events
            if(currGamepad.a && !prevGamepad.a){
                liftState++;
                if(liftState > 1){
                    liftState = 0;
                }

                switch (liftState){
                    case 0:
                        intake.closePlacer();
                        intake.openClaw();
                        lift.setHeight(0);
                        break;
                    case 1:
                        lift.setHeight(1);
                        break;
                }
            }

            //Handle Intake Events
            if(currGamepad.x && !prevGamepad.x){
                intakeState++;
                if(intakeState > 2){
                    intakeState = 0;
                }

                switch (intakeState){
                    case 0:
                        intake.wristUp();
                        intake.setExtension(0);
                        break;
                    case 1:
                        intake.wristMid();
                        intake.setExtension(1);
                        break;
                    case 2:
                        intake.wristDown();
                        intake.setExtension(1);
                        break;
                }
            }

            //Handle Claw Events
            if(currGamepad.b && !prevGamepad.b){
                clawState++;
                if(clawState > 2){
                    clawState = 0;
                }

                switch (clawState){
                    case 0:
                        intake.closePlacer();
                        intake.openClaw();
                        break;
                    case 1:
                        intake.openPlacer();
                        intake.closeClaw();
                        break;
                }
            }

            lift.update();
            intake.update();

            //Telemetry
            telemetry.addData("Status: ", "Active");
            telemetry.addLine();
            robot.bufferTelemetry();
            lift.bufferTelemetry();
            intake.bufferTelemetry();
            telemetry.update();
        }
    }
}
