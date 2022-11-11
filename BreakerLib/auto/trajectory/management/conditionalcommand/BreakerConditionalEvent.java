// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** A base interface for all classes that represent a time or pose triggered event that can be attached to a {@link BreakerTrajectoryPath} */
public interface BreakerConditionalEvent {
    public abstract boolean checkCondition(double currentTimeSeconds, Pose2d currentPose);
    public abstract Command getBaseCommand();
}
