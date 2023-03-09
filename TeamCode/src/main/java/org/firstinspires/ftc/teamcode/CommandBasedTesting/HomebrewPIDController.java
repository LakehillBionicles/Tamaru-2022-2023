package org.firstinspires.ftc.teamcode.CommandBasedTesting;

public class HomebrewPIDController {

    private double kP, kI, kD;
    private double target, current;
    private double integralSumLimit;
    private double error;

    private double tolerance = 0.5;

    /**
     * base constructor
     */
    public HomebrewPIDController(double kp, double ki, double kd){
        this(kp, ki, kd, 0, 0);
    }

    /**
     * full contructor
     * @param reference target position
     * @param state current position
     */
    public HomebrewPIDController(double kp, double ki, double kd, double reference, double state){
        kP = kp;
        kI = ki;
        kD = kd;

        target = reference;
        current = state;

        integralSumLimit = 0.25;

        error = target - current;
    }

    /**
     * sets the tolerance (acceptable error)
     * @param positionTolerance tolerable error
     */
    public void setTolerance(double positionTolerance){
        tolerance = positionTolerance;
    }

    /**
     * returns the current target of PIDController
     * @return the current target
     */
    public double getTarget() {
        return target;
    }

    /**
     * sets the target of the PIDController
     * @param targetPosition the desired target
     */
    public void setTarget(double targetPosition) {
        target = targetPosition;
        error = target - current;
    }

    /**
     * boolean to determine if current error is within tolerance
     * @return true if error is within tolerance, false if not
     */
    public boolean atTarget() {
        return Math.abs(error) < tolerance;
    }

    /**
     * returns position error
     * @return current position error
     */
    public double getError() {
        return error;
    }

    /**
     * sets the gains for the controller
     */
    public void setPID(double kp, double ki, double kd) {
        kP = kp;
        kI = ki;
        kD = kd;
    }


}
