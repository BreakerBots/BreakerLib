// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.selftest;

/** Add your docs here. */
public enum DeviceHealth {
    /** Device is functioning entirely or mostly within expected perameters. */
    NOMINAL,
    /** Device has encountered one or more problems but can still function. */
    FAULT,
    /** Device has encounterd one or more fatal errors, meaning the device is no longer in working condition. */
    INOPERABLE
}
