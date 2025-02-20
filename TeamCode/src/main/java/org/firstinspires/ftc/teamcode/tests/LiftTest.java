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
@Disabled
public class LiftTest extends LinearOpMode {
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

        Lift lift = new Lift(armMotor, dumbMotor,7000,this);
        lift.setPidConstant(0, 0, 0, 0,0);

        Gamepad currentGamepad = new Gamepad();
        Gamepad prevGamepad = new Gamepad();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
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
                armMotor.setPower(-gamepad1.left_stick_y);
                dumbMotor.setPower(-gamepad1.left_stick_y);
            }

            lift.setPidConstant(KP, KI, KD, A, MAX_INTEGRAL);
            lift.setGravityOffset(KG);
            lift.update();

            telemetry.addData("Status: ", "ACTIVE");
            lift.bufferTelemetry();
            telemetry.update();
        }
    }
}
