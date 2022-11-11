// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerTimeTriggeredEvent implements BreakerConditionalEvent {
    private double triggerTime;
    private Command commandToRun;

    public BreakerTimeTriggeredEvent(double triggerTime, Command commandToRun) {
        this.commandToRun = commandToRun;
        this.triggerTime = triggerTime;
    }

    @Override
    public Command getBaseCommand() {
        return commandToRun;
    }

    @Override
    public boolean checkCondition(double currentTimeSeconds, Pose2d currentPose) {
        return triggerTime >= currentTimeSeconds;
    }
}
