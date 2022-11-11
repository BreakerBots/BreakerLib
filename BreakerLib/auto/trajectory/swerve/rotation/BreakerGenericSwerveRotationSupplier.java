// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.rotation;

import edu.wpi.first.math.geometry.Rotation2d;

/** A base interface for all classes capabe of supplying live rotation setpoint information to a swerve trajectory follower */
public interface BreakerGenericSwerveRotationSupplier {
    public abstract Rotation2d getRotation(double currentTime);
}
