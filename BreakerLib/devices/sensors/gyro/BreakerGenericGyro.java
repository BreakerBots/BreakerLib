// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/** A base interface for all single axis gyroscopes and gyroscopes capable of Yaw readings */
public interface BreakerGenericGyro {

    /** @return Yaw angle within +-180 degrees. */
    public abstract double getYawDegrees();

    /** @return Yaw angle as {@link Rotation2d} within +-180 degrees. */
    public abstract Rotation2d getYawRotation2d();

    /** Sets yaw to angle value. */
    public abstract void setYaw(double value);

    /** @return Biased angular velocity yaw.(deg/sec) */
    public abstract double getYawRate();

    /** Resets all angles avalible to 0 degrees */
    public abstract void reset();

    /** @return Raw yaw value without modulus. */
    public abstract double getRawYaw();

    /** @return Raw angular velocity yaw. (deg/sec) */
    public abstract double getRawYawRate();
    

}
