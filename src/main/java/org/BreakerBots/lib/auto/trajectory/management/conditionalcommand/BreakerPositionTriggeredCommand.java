// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerPositionTriggeredCommand implements BreakerConditionalCommand{
    private Supplier<Pose2d> currentPoseSupplier;
    private Pose2d triggerPose;
    private Pose2d tolerences;
    private Command commandToRun;
    public BreakerPositionTriggeredCommand(Pose2d triggerPose, Supplier<Pose2d> currentPoseSupplier, Pose2d triggerPoseTolerences, Command commandToRun) {
        this.currentPoseSupplier = currentPoseSupplier;
        this.commandToRun = commandToRun;
        this.triggerPose = triggerPose;
        triggerPoseTolerences = tolerences;
    }

    @Override
    public boolean checkCondition() {
        boolean satX = BreakerMath.isRoughlyEqualTo(triggerPose.getX(), currentPoseSupplier.get().getX(), Math.abs(tolerences.getX()));
        boolean satY = BreakerMath.isRoughlyEqualTo(triggerPose.getY(), currentPoseSupplier.get().getY(), Math.abs(tolerences.getY()));
        boolean satRot = BreakerMath.isRoughlyEqualTo(triggerPose.getRotation().getDegrees(), currentPoseSupplier.get().getRotation().getDegrees(), Math.abs(tolerences.getRotation().getDegrees()));
        return satX && satY && satRot;
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
