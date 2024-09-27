package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.IMU;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * FTC 2024-2025 Into The Deep Tank Drive API
 * <br/>Sept 27th, 2024
 * @author Connor Feeney
 */
public class DriveTrain {
    //Enums for simplifying setup
    public enum PoweredDir {
        pos("Positive"),
        neg("Negative");

        private final String displayName;
        PoweredDir(String displayName) {
            this.displayName = displayName;
        }

        @NonNull
        @Override
        public String toString(){
            return  displayName;
        }
    }
    public enum Side {
        Left("Left"),
        Right("Right");


        private final String displayName;
        Side(String displayName) {
            this.displayName = displayName;
        }

        @NonNull
        @Override
        public String toString(){
            return  displayName;
        }
    }

    private static class MotorConfigPair {
        private final PoweredDir dir;
        private final Side side;
        public MotorConfigPair(PoweredDir dir, Side side){
            this.dir = dir;
            this.side = side;
        }

        public PoweredDir getDir(){
            return dir;
        }

        public Side getSide() {
            return side;
        }
    }

    private float wheelDiam;
    private float RPM;
    private IMU imu;
    private LinearOpMode opMode;
    private volatile float L_power = 0f; //Power of left side motors
    private volatile float R_power = 0f; //Powers of right side motors
    private float speedModifier = 1.0f; //Modifier to max speed
    private final HashMap<DcMotor, MotorConfigPair> motors; //All Drive motors

    public DriveTrain(){
        motors = new HashMap<>();
    }

    /*===== DriveTrain Config Functions =====*/
    
    /**
     * Add a motor to the drive train
     *
     * @param motor The DcMotor object you will be using.
     *
     * @param dir The logical powered direction of the motor you will be using,
     *            Depending on motor orientation the logical powered direction will be different,
     *            in one orientation "pos" may cause the motor to move "forward" while "neg" will
     *            cause the motor to move backwards, the easiest way to know what to set this to is
     *            to set all motors to "pos" and see which motors move the proper direction and flip
     *            the values for the motors accordingly.
     *
     * @param side the side of the robot which the motor is one from the POV of the back of the robot.
     */
    public void addMotor(DcMotor motor, PoweredDir dir, Side side) {
        motors.put(motor, new MotorConfigPair(dir, side));
    }

    /**
     * allows you to programmatically change the max speed of the robot
     * setting this to zero will make the robot never move
     *
     * @param speedModifier maximum speed of DriveTrain (0f - 1.0f)
     */
    public void setSpeedModifier(float speedModifier){
        this.speedModifier = clip(speedModifier, 0f, 1.0f);
    }

    /*===== DriveTrain TeleOp Functions =====*/

    /**
     * Make the DriveTrain drive based on the gamepad stick values.
     *
     * @param LS_pitch Raw pitch or y-value of the left gamepad stick (gamepad1.left_stick_y)
     * @param RS_yawn Raw yawn or x-value of the right gamepad stick (gamepad1.right_stick_x)
     */
    public void drive(float LS_pitch, float RS_yawn) {
        L_power = clip(-LS_pitch + RS_yawn, -1.0f, 1.0f) * speedModifier;
        R_power = clip(-LS_pitch - RS_yawn, -1.0f, 1.0f) * speedModifier;

        for(DcMotor motor : motors.keySet()){
            MotorConfigPair config = motors.get(motor);

            assert config != null;
            if(config.getSide() == Side.Left){
                motor.setPower(config.getDir() == PoweredDir.pos ? L_power : -L_power);
            }else{
                motor.setPower(config.getDir() == PoweredDir.pos ? R_power : -R_power);
            }
        }
    }

    /*===== DriveTrain Autonomous Functions =====*/

    /**
     * Setup all required parameters for drivetrain autonomous functions, must be called before using any autonomous functions
     *
     * @param wheelDiam Diameter of powered wheels
     * @param RPM Calculated RPM of drive motors, including adjustments for gear ratios
     * @param logoDir The direction the Rev-Control-Hub logo is facing relative to the robot
     * @param usbDir The direction the Rev-Control-Hub usb ports are facing to relative to the robot
     * @param opMode The running OpMode (just input "this")
     */
    public void autoInit(float wheelDiam, float RPM, RevHubOrientationOnRobot.LogoFacingDirection logoDir, RevHubOrientationOnRobot.UsbFacingDirection usbDir, LinearOpMode opMode){
        this.wheelDiam = wheelDiam;
        this.RPM = RPM;

        this.imu = hardwareMap.get(IMU.class, "imu");
        this.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDir, usbDir)));
        this.imu.resetYaw();

        this.opMode = opMode;
    }

    /**
     *
     * @param angle angle
     */
    public void autoTurn(double angle) {
        assert imu != null;
    }

    /**
     *
     * @param dist distance
     */
    public void autoDrive(float dist) {
        assert wheelDiam > 0;
        assert RPM > 0;
    }

    /*===== Util Functions =====*/

    /**
     * Display DriveTrain telemetry on your driver hub,
     * you still need to call telemetry.update() after calling this function,
     * this function only adds the data to the stack it does not update it to
     * avoid telemetry update conflicts
     *
     * @param telemetry Your OpMode telemetry object
     */
    public void updateTelemetry(Telemetry telemetry){
        telemetry.addData("Left Side Power: ", L_power);
        telemetry.addData("Right Side Power", R_power);
    }

    /**
     * Extended more detailed version of {@link #updateTelemetry(Telemetry telemetry)},
     * less performant should only be used for debug.
     *
     * @param telemetry Your OpMode telemetry object
     * @see #updateTelemetry(Telemetry telemtry)
     */
    public void updateTelemetryEX(Telemetry telemetry){
        updateTelemetry(telemetry);

        telemetry.addData("X-Rotation: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        final ArrayList<DcMotor> keys = new ArrayList<>(motors.keySet());
        for(int i = 0; i < keys.size(); i++){
            DcMotor motor = keys.get(i);
            MotorConfigPair config = motors.get(motor);

            assert config != null;
            telemetry.addData("Motor " + i + " | ", "Side: %s | PoweredDir: %s | Power: %f", config.getSide().toString(), config.getDir().toString(), motor.getPower());
        }
    }

    /**
     * Clips a value if it is over or under a set minimum or maximum value
     *
     * @param value Initial Value
     * @param min Minimum possible value
     * @param max Max possible value
     * @return Clipped value
     */
    private float clip(float value, float min, float max){
        if(value >= max){
            return max;
        } else if (value <= min) {
            return min;
        }

        return value;
    }
}