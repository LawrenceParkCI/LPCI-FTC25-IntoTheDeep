package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.calibration.util.PIDAutoTuner;
import org.firstinspires.ftc.teamcode.components.intake.Intake;
import org.firstinspires.ftc.teamcode.components.lift.Lift;

@Autonomous(name="Robot Calibration", group = "Calibration")
public class RobotCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Lift Components
        DcMotor liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        DcMotor liftMotorD = hardwareMap.get(DcMotor.class, "liftMotorD");

        //Intake Components
        DcMotor controlMotor = hardwareMap.get(DcMotor.class, "controlMotor");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        Servo wristL = hardwareMap.get(Servo.class, "wristL");
        Servo wristR = hardwareMap.get(Servo.class, "wristR");
        Servo placer = hardwareMap.get(Servo.class, "placer");

        //Robot Components
        Lift lift = new Lift(liftMotor, liftMotorD, 7000, this);
        Intake intake = new Intake(claw, wristL, wristR, placer, controlMotor, 220 ,this);

        telemetry.addData("Status: ", "INIT");
        telemetry.addLine("Play to Tune");
        telemetry.update();
        waitForStart();

        //Auto tune robot components
        PIDAutoTuner liftTuner = new PIDAutoTuner(liftMotor, liftMotorD);
        PIDAutoTuner intakeTuner = new PIDAutoTuner(controlMotor);
        if(opModeIsActive()){
            liftTuner.tune(5, 0.5, lift.getMaxEncoderValue(), 1,this);
             intakeTuner.tune(5, 0.5, intake.getMaxEncoderValue(), 0.5,this);
        }

        //Output calibrated data
        while (opModeIsActive()){
            telemetry.addData("Status: ", "Tuned");
            telemetry.addData("Lift PID: ", "(" + liftTuner.getKP() + ", " + liftTuner.getKI() + ", " + liftTuner.getKD() + ")");
            telemetry.addData("Intake PID: ", "(" + intakeTuner.getKP() + ", " + intakeTuner.getKI() + ", " + intakeTuner.getKD() + ")");
            telemetry.update();
        }
    }
}
