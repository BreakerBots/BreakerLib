// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.swervemodules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;

/** Interface for all Swerve Modules to allwo for easy interchangeablity, this class is meant to surve as an intermedairy between your swerve hardware and the BreakerSwerveDrive class */
public interface BreakerGenericSwerveModule {

    /** default method for setting a swerve module to a given target state, 
     * automaticly calls the overloded version of this method that independently specifyes angle and speed*/
    public default void setModuleTarget(SwerveModuleState targetModuleState) {
        setModuleTarget(targetModuleState.angle, targetModuleState.speedMetersPerSecond);
    }

    public abstract void setModuleTarget(Rotation2d targetAngle, double targetVelocityMetersPerSecond);

    public abstract double getModuleAbsoluteAngle();

    public abstract double getModuleRelativeAngle();
    
    public abstract double getModuleVelMetersPerSec();

    public abstract double getMetersPerSecToFalconRSU(double speedMetersPerSec);

    public abstract SwerveModuleState getModuleState();

    public abstract boolean atAngleSetpoint();

    public abstract boolean atVelSetpoint();

    public abstract boolean atSetModuleState();

    public abstract void runModuleSelfCheck();

    public abstract void setDriveMotorBrakeMode(boolean isEnabled);

    public abstract void setTurnMotorBreakMode(boolean isEnabled);

    /** returns the modules health as an array [0] = overall, [1] = drive motor, [2] = turn motor, [3] = outher if supported (EX: CANCoder)*/
    public abstract DeviceHealth[] getModuleHealth();

    public abstract boolean moduleHasFault();

    public abstract String getModuleFaults();
}
