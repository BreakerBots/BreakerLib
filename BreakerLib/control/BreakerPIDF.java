// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** PIDF controller. */
public class BreakerPIDF {
    private PIDController pidController;
    private SimpleMotorFeedforward ffController;
    private double pidMinOutput, pidMaxOutput;

    /**
     * Makes a BreakerPIDF with no minimum or maximum value.
     * 
     * @param pidController PID controller controller.
     * @param ffController  Feedforward controller.
     */
    public BreakerPIDF(PIDController pidController, SimpleMotorFeedforward ffController) {
        this.pidController = pidController;
        this.ffController = ffController;
        pidMinOutput = -Double.MAX_VALUE;
        pidMaxOutput = -Double.MIN_VALUE;
    }

    /**
     * Makes a BreakerPIDF with chosen minimum and maximum outputs.
     * 
     * @param pidController PID controller controller.
     * @param ffController  Feedforward controller.
     * @param pidMinOutput  Minimum PID output.
     * @param pidMaxOutput  Maximum PID output.
     */
    public BreakerPIDF(PIDController pidController, SimpleMotorFeedforward ffController, double pidMinOutput,
            double pidMaxOutput) {
        this.pidController = pidController;
        this.ffController = ffController;
        this.pidMaxOutput = pidMaxOutput;
        this.pidMinOutput = pidMinOutput;
    }

    /** Set velocity and acceleration tolerance for the PID.
     * 
     * @param velocityTolerence Velocity tolerance.
     * @param accelerationTolerence Acceleration tolerance.
     */
    public void setTolerence(double velocityTolerence, double accelerationTolerence) {
        pidController.setTolerance(velocityTolerence, accelerationTolerence);
    }

    /** Calculates motor output value based on PID and feedforward calculations. */
    public double calculate(double measurement, double setpoint) {
        return MathUtil.clamp(pidController.calculate(measurement, setpoint), pidMinOutput, pidMaxOutput)
                + ffController.calculate(setpoint);
    }

    // public double calculatePrecentSpeed(double measurement, double setPoint,
    // double voltage) {
    // return pidController.calculate(measurement, setPoint) +
    // (ffController.calculate(setPoint) / voltage);
    // }

    /** Returns true if at PID setpoint.*/
    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    /** Gets... position error? (DAMMIT ROMAN!) */
    public double getVelocityError() {
        return pidController.getPositionError();
    }

    /** Returns feedforward controller. */
    public SimpleMotorFeedforward getBaseFeedforwardController() {
        return ffController;
    }

    /** Returns PID controller. */
    public PIDController getBasePidController() {
        return pidController;
    }
}
