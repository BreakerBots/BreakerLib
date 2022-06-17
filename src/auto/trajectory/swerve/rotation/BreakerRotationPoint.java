// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.rotation;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/** Add your docs here. */
public class BreakerRotationPoint {
    private Rotation2d rotation;
    private double timeOfRotation;
    /** creates a new BreakerRotationPoint
     * @param rotation - the robot rotation
     * @param timeOfRotation - the time along the trajectory in seconds that this rotation represents
     */
    public BreakerRotationPoint(Rotation2d rotation, double timeOfRotation) {
        this.rotation = rotation;
        this.timeOfRotation = timeOfRotation;
    }

    public Rotation2d getRotation() {
        return rotation;
    }

    public double getTimeOfRotation() {
        return timeOfRotation;
    }
}
