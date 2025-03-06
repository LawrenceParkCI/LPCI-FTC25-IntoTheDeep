package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.lift.Lift;

@TeleOp(name = "Lift Test", group = "Test")
@Config
public class LiftTest extends LinearOpMode {
    public static double KP = 0;
    public static double KI = 0;
    public static double KD = 0;

    public static double A = 0;
    public static double MAX_INTEGRAL = 0;

    public static double KG = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        DcMotor liftMotorD = hardwareMap.get(DcMotor.class, "liftMotorD");

        Lift lift = new Lift(liftMotor, liftMotorD,7000,this);
        lift.setPIDConstant(0, 0, 0, 0,0);

        Gamepad currentGamepad = new Gamepad();
        Gamepad prevGamepad = new Gamepad();
        telemetry.addData("Status: ", "INIT");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            prevGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if(currentGamepad.a && !prevGamepad.a){
                lift.setHeight(0.5);
            }
            if(currentGamepad.b && !prevGamepad.b){
                lift.setHeight(0);
            }
            if(currentGamepad.x && !prevGamepad.x){
                lift.setHeight(1);
            }

            if(currentGamepad.dpad_up){
                liftMotor.setPower(-gamepad1.left_stick_y);
                liftMotorD.setPower(-gamepad1.left_stick_y);
            }

            lift.setPIDConstant(KP, KI, KD, A, MAX_INTEGRAL);
            lift.setGravityOffset(KG);
            lift.update();

            telemetry.addData("Status: ", "ACTIVE");
            lift.bufferTelemetry();
            telemetry.update();
        }
    }
}
