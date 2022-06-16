// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.powermanagement;

/** Add your docs here. */
public class BreakerPowerManagementConfig {
    private BreakerPowerChannel[] attachedPowerChannels;
    private PowerManagementPriority managementPriority;
    public BreakerPowerManagementConfig(PowerManagementPriority managementPriority, BreakerPowerChannel... attachedPowerChannels) {
        this.managementPriority = managementPriority;
        this.attachedPowerChannels = attachedPowerChannels;
    }

    public BreakerPowerChannel[] getAttachedPowerChannels() {
        return attachedPowerChannels;
    }

    public PowerManagementPriority getPowerManagementPriority() {
        return managementPriority;
    }
}
