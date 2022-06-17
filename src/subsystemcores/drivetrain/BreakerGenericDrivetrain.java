// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.selftest.BreakerSelfTestable;

/** Contianer class for methods common to all drivetrain types */
public interface BreakerGenericDrivetrain extends BreakerGenericOdometer, BreakerGenericDevice {
    
    /** Updates the odometer position. */
    public abstract void updateOdometry();

    public abstract void setDrivetrainBrakeMode(boolean isEnabled);

    public abstract void setSlowMode(boolean isEnabled);

    public abstract boolean isInSlowMode();

    public abstract ChassisSpeeds getFieldRelativeChassisSpeeds(BreakerGenericOdometer odometer);

}
