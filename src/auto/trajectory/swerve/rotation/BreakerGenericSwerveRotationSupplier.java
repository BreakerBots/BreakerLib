// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.rotation;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface BreakerGenericSwerveRotationSupplier {
    public abstract void setCurrentTime(double currentTime);

    public abstract BreakerRotationPoint[] getRotationPoints();

    public abstract Rotation2d getRotation();
}
