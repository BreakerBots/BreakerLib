// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision;

/** Add your docs here. */
public interface BreakerVisionTargetData {
    public abstract double getYaw();
    public abstract double getPitch();
    public abstract double getTargetDistance();
    public abstract double getArea();
    public abstract double getSkew();
}
