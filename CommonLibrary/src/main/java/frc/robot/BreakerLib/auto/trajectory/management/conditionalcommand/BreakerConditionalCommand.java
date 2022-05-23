// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public interface BreakerConditionalCommand {
    public abstract boolean checkCondition();
    public abstract Command getBaseCommand();
    public abstract void startRunning();
    public abstract boolean updateAutoRun();
}
