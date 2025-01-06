package org.firstinspires.ftc.teamcode.components.intake;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.lift.Lift;

public class Intake {

    /**
     * Record the state of the intake
     */
    enum State{
        Neutral,
        Grabbing,
        Transferring,
        Holding,
        Placing
    }

    public Intake(Lift lift, DcMotor controlMotor, LinearOpMode opMode, Servo... servos) {

    }

}
