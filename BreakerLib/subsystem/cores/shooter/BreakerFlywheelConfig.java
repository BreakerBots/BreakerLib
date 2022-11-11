// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Add your docs here. */
public class BreakerFlywheelConfig {
    private double kP;
    private double kI;
    private double kD;
    private double kF;
    private double velocityTolerence;
    private double acclerationTolerence;
    private BreakerArbitraryFeedforwardProvider arbFFProvider;

    public BreakerFlywheelConfig(double kP, double kI, double kD, double kF, double velocityTolerence, double accelerationTolerence, BreakerArbitraryFeedforwardProvider arbFFProvider) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.velocityTolerence= velocityTolerence;
        this.acclerationTolerence = accelerationTolerence;
        this.arbFFProvider = arbFFProvider;
    }

    public double getkD() {
        return kD;
    }
    public double getkF() {
        return kF;
    }

    public double getkI() {
        return kI;
    }

    public double getkP() {
        return kP;
    }

    public double getAcclerationTolerence() {
        return acclerationTolerence;
    }

    public double getVelocityTolerence() {
        return velocityTolerence;
    }

    public BreakerArbitraryFeedforwardProvider getArbFFProvider() {
        return arbFFProvider;
    }
}
