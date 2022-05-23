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
public class BreakerTimeTriggeredCommand implements BreakerConditionalCommand{
    private DoubleSupplier currentTimeSupplier;
    private double triggerTime;
    private double tolerences;
    private Command commandToRun;
    public BreakerTimeTriggeredCommand(double triggerTime, DoubleSupplier currentTimeSupplier, double triggerTimeTolerences, Command commandToRun) {
        this.currentTimeSupplier = currentTimeSupplier;
        this.commandToRun = commandToRun;
        this.triggerTime = triggerTime;
        triggerTimeTolerences = tolerences;
    }

    @Override
    public boolean checkCondition() {
        return BreakerMath.isRoughlyEqualTo(triggerTime, currentTimeSupplier.getAsDouble(), tolerences);
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
    public boolean updateAutoRun() {
        if (checkCondition()) {
            startRunning();
            return true;
        }
        return false;
    }
}
