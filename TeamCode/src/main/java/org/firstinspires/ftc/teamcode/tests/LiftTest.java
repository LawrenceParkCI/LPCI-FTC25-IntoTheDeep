package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.intake.Intake;
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
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift lift = new Lift(armMotor, 1,this);

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
                lift.setHeight(1.0);
            }
            if(currentGamepad.b && !prevGamepad.b){
                lift.setHeight(0);
            }
            if(currentGamepad.x && !prevGamepad.x){
                lift.setHeight(0.5);
            }

            lift.setPidConstant(KP, KI, KD, A, MAX_INTEGRAL);
            lift.setGravityOffset(KG);
            lift.update();

            armMotor.setPower(-gamepad1.left_stick_y);

            telemetry.addData("Status: ", "ACTIVE");
            lift.bufferTelemetry();
            telemetry.update();
        }
    }
}
