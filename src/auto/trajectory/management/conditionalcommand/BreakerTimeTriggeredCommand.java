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
public class BreakerTimeTriggeredCommand implements BreakerConditionalCommand {
    private DoubleSupplier currentTimeSupplier;
    private double triggerTime;
    private double tolerences;
    private Command commandToRun;
    private boolean usesSupplier;

    public BreakerTimeTriggeredCommand(double triggerTime, double triggerTimeTolerences, Command commandToRun) {
        this.commandToRun = commandToRun;
        this.triggerTime = triggerTime;
        triggerTimeTolerences = tolerences;
        usesSupplier = false;
    }
    public BreakerTimeTriggeredCommand(double triggerTime, DoubleSupplier currentTimeSupplier, double triggerTimeTolerences, Command commandToRun) {
        this.currentTimeSupplier = currentTimeSupplier;
        this.commandToRun = commandToRun;
        this.triggerTime = triggerTime;
        triggerTimeTolerences = tolerences;
        usesSupplier = true;
    }

    @Override
    public Command getBaseCommand() {
        return commandToRun;
    }
    @Override
    public void startRunning() {
        commandToRun.schedule();
    }

    @Override
    public boolean checkCondition(double currentTimeSeconds, Pose2d currentPose) {
        return BreakerMath.isRoughlyEqualTo(triggerTime, usesSupplier ? currentTimeSupplier.getAsDouble() : currentTimeSeconds, tolerences);
    }

    @Override
    public boolean updateAutoRun(double currentTimeSeconds, Pose2d currentPose) {
        if (checkCondition(currentTimeSeconds, currentPose)) {
            startRunning();
            return true;
        }
        return false;
    }
}
