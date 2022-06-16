// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;

/** Interface for all BreakerLib classes capable of generteing odometry data */
public interface BreakerGenericOdometer {
  public abstract void setOdometryPosition(Pose2d newPose);

  public abstract Object getBaseOdometer();

  public abstract Pose2d getOdometryPoseMeters();

  public abstract BreakerMovementState2d getMovementState();
  
  public abstract ChassisSpeeds getRobotRelativeChassisSpeeds();
    
  public abstract ChassisSpeeds getFieldRelativeChassisSpeeds();
}
