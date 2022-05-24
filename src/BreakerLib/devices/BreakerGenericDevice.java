 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import frc.robot.BreakerLib.util.selftest.DeviceHealth;

/** Base interface for all Breaker devices. */
public interface BreakerGenericDevice {
    public abstract void runSelfTest();

    public abstract DeviceHealth getHealth();

    public abstract String getFaults();

    public abstract String getDeviceName();

    public abstract boolean hasFault();

    public abstract void setDeviceName(String newName);
}
