// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.power;

/** Add your docs here. */
public enum PowerManagementPriority {
    LOW_PRIORITY,
    MEDIUM_PRIORITY,
    HIGH_PRIORITY,
    /** For robot systems or devices that are aboslotely nessicery for the robot's opperation, will never be fully deactivated and rairly ever throttled */
    CRITICAL
}
