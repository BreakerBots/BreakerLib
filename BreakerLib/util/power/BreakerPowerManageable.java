// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.power;

/** Add your docs here. */
public interface BreakerPowerManageable {
    public abstract DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig);
    public abstract void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode);
    public abstract void returnToAutomaticPowerManagement();
    public abstract boolean isUnderAutomaticControl();
    public abstract DevicePowerMode getPowerMode();
}
