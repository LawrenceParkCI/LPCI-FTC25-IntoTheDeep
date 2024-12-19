package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.components.intake.Intake;
import org.firstinspires.ftc.teamcode.components.lift.Lift;

@TeleOp(name = "Intake Test", group = "Test")
@Config
@Disabled
public class IntakeTest extends LinearOpMode {
    public static double KP = 0;
    public static double KI = 0;
    public static double KD = 0;

    public static double A = 0;
    public static double MAX_INTEGRAL = 0;

    public static double KG = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        DcMotor dumbMotor = hardwareMap.get(DcMotor.class, "dumbMotor");

        Lift lift = new Lift(armMotor, dumbMotor,6800,this);
        lift.setPidConstant(6.5, 20, 0, 0,0);
        lift.setHeight(0.5);

        CRServo rackServo = hardwareMap.get(CRServo.class, "rackServo");
        CRServo intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        TouchSensor extendedSensor = hardwareMap.get(TouchSensor.class, "extendedSensor");
        TouchSensor retractedSensor = hardwareMap.get(TouchSensor.class, "retractedSensor");

        Intake intake = new Intake(intakeServo, rackServo, intakeMotor, extendedSensor, retractedSensor, 1440, this);

        Gamepad currentGamepad = new Gamepad();
        Gamepad prevGamepad = new Gamepad();

        intake.setPosition(20, false);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry.addData("Status: ", "INIT");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            prevGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if(currentGamepad.a && !prevGamepad.a){
                intake.setPosition(20,false);
            }
            if(currentGamepad.b && !prevGamepad.b){
                intake.setPosition(-10,true);
            }

            if(currentGamepad.dpad_up){
                intakeMotor.setPower(-gamepad1.left_stick_y);
            }

            intake.setPidConstant(KP, KI, KD, A, MAX_INTEGRAL);
            intake.setGravityOffset(KG);

            intake.update();
            lift.update();

            telemetry.addData("Status: ", "ACTIVE");
            intake.bufferTelemetry();
            telemetry.update();
        }
    }
}
