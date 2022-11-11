// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * Interface for all Swerve Modules to allow for easy interchangeablity, this
 * class is meant to serve as an intermedairy between your swerve hardware and
 * the BreakerSwerveDrive class
 */
public interface BreakerGenericSwerveModule extends BreakerGenericDevice {

    /**
     * default method for setting a swerve module to a given target state,
     * automaticly calls the overloded version of this method that independently
     * specifyes angle and speed
     */
    public default void setModuleTarget(SwerveModuleState targetModuleState) {
        setModuleTarget(targetModuleState.angle, targetModuleState.speedMetersPerSecond);
    }

    /** Sets the modules target speed to zero while maintaning the last set angle */
    public default void stop() {
        setModuleTarget(getModuleTargetState().angle, 0.0);
    }

    /**
     * Method defined in module code to handle angle and velocity control of the
     * module
     */
    public abstract void setModuleTarget(Rotation2d targetAngle, double targetVelocityMetersPerSecond);

    /** @return the absolute (+/- 180 deg) angle of the module in degrees */
    public abstract double getModuleAbsoluteAngle();

    /**
     * @return the relative (with rollover, 180 -> 181) angle of the module in
     *         degrees
     */
    public abstract double getModuleRelativeAngle();

    /** @return The velocity of the module's drive wheel in meters per second. */
    public abstract double getModuleVelMetersPerSec();

    /** @return The velocity of the module's drive wheel in device's native unit. */
    public abstract double getMetersPerSecToNativeVelUnits(double speedMetersPerSec);

    public abstract SwerveModuleState getModuleTargetState();

    /** @return Module's {@link SwerveModuleState}. */
    public abstract SwerveModuleState getModuleState();

    /**
     * Sets brake mode on drive motor.
     * 
     * @param isEnabled Whether or not to enable brake mode.
     */
    public abstract void setDriveMotorBrakeMode(boolean isEnabled);

    /**
     * Sets brake mode on turn motor.
     * 
     * @param isEnabled Whether or not to enable brake mode.
     */
    public abstract void setTurnMotorBrakeMode(boolean isEnabled);

    /**
     * Sets brake mode on drive and turn motors.
     * 
     * @param isEnabled Whether or not to enable brake mode.
     */
    public abstract void setModuleBrakeMode(boolean isEnabled);

    /**
     * @return Module's health as an array.
     *         <p>
     *         [0] = overall, [1] = drive motor, [2] = turn motor, [3] = other device if
     *         supported (EX: CANCoder)
     */
    public abstract DeviceHealth[] getModuleHealths();

    public static String getModuleAsString(String moduleType, BreakerGenericSwerveModule module) {
        return String.format("%s(Name: %s, Overall_Device_Health: %s, Set_State: %s, Current_State: %)", moduleType, module.getDeviceName(), module.getHealth().toString(), module.getModuleTargetState().toString(), module.getModuleState().toString());
    }
}
