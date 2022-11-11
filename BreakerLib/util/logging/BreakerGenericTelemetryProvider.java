// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.logging;

/** Add your docs here. */
public interface BreakerGenericTelemetryProvider {
    public abstract String getProviderName();
    public abstract void setProviderName(String newName);
    public abstract boolean isTelemetryEnabled();
    public abstract void setTelemetryEnabled(boolean isEnabled);

    public default void logTelemetry(String data) {
        BreakerTelemetry.logTelemetry(isTelemetryEnabled(), getProviderName(), data);
    }
}
