package org.firstinspires.ftc.teamcode.calibration.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class PIDAutoTuner {
    private final DcMotor m;
    private final DcMotor[] d;

    private double KP = 0;
    private double KI = 0;
    private double KD = 0;

    public PIDAutoTuner(DcMotor m, DcMotor... d) {
        this.m = m;
        this.d = d;

        this.m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for(DcMotor motor : this.d){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void tune(int relayCount, double setPoint, double max, double amplitude, LinearOpMode opMode) throws IllegalArgumentException {
        if(relayCount <= 0){
            throw new IllegalArgumentException("relayCount must be > 0");
        }

        if(setPoint >= 1){
            throw new IllegalArgumentException("setPoint should be < 1");
        }

        if(max <= 0){
            throw new IllegalArgumentException("Max must be > 1");
        }

        final List<Double> highPeakTimes = new ArrayList<>();
        final List<Double> highPeakPositions = new ArrayList<>();

        final List<Double> lowPeakTimes = new ArrayList<>();
        final List<Double> lowPeakPositions = new ArrayList<>();

        final double h = 20;

        double relayOutput = amplitude;

        boolean relaySateHigh = true;
        m.setPower(relayOutput);
        for(DcMotor motor : d) {
            motor.setPower(relayOutput);
        }

        final ElapsedTime runtime = new ElapsedTime();

        while (opMode.opModeIsActive() && (highPeakTimes.size() < relayCount || lowPeakTimes.size() < relayCount)) {
            double currentPosition = m.getCurrentPosition();
            double currentTime = runtime.seconds();

            if(relaySateHigh && (currentPosition > (setPoint * max) + h)){
                highPeakTimes.add(currentTime);
                highPeakPositions.add(currentPosition / max);

                relayOutput = -amplitude;

                m.setPower(relayOutput);
                for(DcMotor motor : d){
                    motor.setPower(relayOutput);
                }

                relaySateHigh = false;
            } else if(!relaySateHigh && (currentPosition < (setPoint * max) - h)) {
                lowPeakTimes.add(currentTime);
                lowPeakPositions.add(currentPosition / max);

                relayOutput = amplitude;

                m.setPower(relayOutput);
                for(DcMotor motor : d){
                    motor.setPower(relayOutput);
                }

                relaySateHigh = true;
            }
        }

        m.setPower(0);
        for(DcMotor motor : d){
            motor.setPower(0);
        }

        double totalPeriod = 0;
        int periodCount = 0;
        for(int i = 1; i < highPeakTimes.size(); i++){
            double period = highPeakTimes.get(i) - highPeakTimes.get(i - 1);
            totalPeriod += period;
            periodCount++;
        }

        double tU = totalPeriod / periodCount;

        int pairCount = Math.min(highPeakPositions.size(), lowPeakPositions.size());
        double totalAmplitude = 0;
        for(int i = 0; i < pairCount; i++){
            double amp = highPeakPositions.get(i) - lowPeakPositions.get(i);
            totalAmplitude += amp;
        }

        double a = (totalAmplitude / pairCount) / 2.0;

        double kU = (4 * amplitude) / (Math.PI * a);

        KP = 0.2 * kU;
        KI = (0.33 * kU) / tU;
        KD = (0.066 * kU) * tU;
    }

    public double getKP(){
        return KP;
    }

    public double getKI(){
        return KI;
    }

    public double getKD(){
        return KD;
    }
}
