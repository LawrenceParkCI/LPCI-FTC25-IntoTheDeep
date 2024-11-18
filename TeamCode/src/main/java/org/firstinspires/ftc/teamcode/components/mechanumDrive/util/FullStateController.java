package org.firstinspires.ftc.teamcode.components.mechanumDrive.util;

/**
 * FTC Into The Deep 24-25 <br>
 * A state controller can help achieve a specific state of a system, this can be position, velocity or any other state that you can measure.
 * A full state controller can control multiple states at a time, in the case of this class both velocity and position can be controlled
 * <br><br>
 * Last Updated: November 18th, 2024
 * @author Connor Feeney
 */
public class FullStateController {
    double kp;
    double kv;

    public FullStateController(double kp, double kv){
        this.kp = kp;
        this.kv = kv;
    }

    public double calculate(double currentPosition, double targetPosition, double currentVelocity, double targetVelocity){
        double positionError = targetPosition - currentPosition;
        double velocityError = targetVelocity - currentVelocity;

        return (kp * positionError) + (kv * velocityError);
    }
}
