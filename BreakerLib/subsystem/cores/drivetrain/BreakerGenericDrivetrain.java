// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.devices.BreakerGenericLoopedDevice;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;

/** Contianer class for methods common to all drivetrain types */
public abstract class BreakerGenericDrivetrain extends BreakerGenericLoopedDevice implements BreakerGenericOdometer {
    protected boolean slowModeActive = false;

    /** Enables the drivetrain's global slow mode if disabled, and disables it if enabled */
    public void toggleSlowMode() {
        slowModeActive = !slowModeActive;
    }

    /**Enables or disables the drivetrians user-configurable "Slow Mode" in 
     * which all drivetrain inputs will be muliplyed by user difigned fractional 
     * constants to allow for figener controll of the robot */
    public void setSlowMode(boolean isEnabled) {
        slowModeActive = isEnabled;
    }

    /** @return Weather or not the drivetrain's global slow mode is enabled */
    public boolean isInSlowMode() {
        return slowModeActive;
    }

    /** @return The {@link BreakerGenericGyro} object used by the drivetrain's intagrated odometry */
    public abstract BreakerGenericGyro getBaseGyro();

    /** Updates the odometer position. */
    public abstract void updateOdometry();

    /** Enables or disables the brake-on-neutral mode of the drivetrains motors */
    public abstract void setDrivetrainBrakeMode(boolean isEnabled);

    /** @return The {@link ChassisSpeeds object that represents } */
    public abstract ChassisSpeeds getFieldRelativeChassisSpeeds(BreakerGenericOdometer odometer);

    /** Stops the robot's movement in all axsies */
    public abstract void stop();

}
