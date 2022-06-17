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
    private boolean usesSupplier;
    public BreakerPositionTriggeredCommand(Pose2d triggerPose, Pose2d triggerPoseTolerences, Command commandToRun) {
        this.commandToRun = commandToRun;
        this.triggerPose = triggerPose;
        tolerences = triggerPoseTolerences;
        usesSupplier = false;
    }

    public BreakerPositionTriggeredCommand(Pose2d triggerPose, Supplier<Pose2d> currentPoseSupplier, Pose2d triggerPoseTolerences, Command commandToRun) {
        this.currentPoseSupplier = currentPoseSupplier;
        this.commandToRun = commandToRun;
        this.triggerPose = triggerPose;
        tolerences = triggerPoseTolerences;
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
        Pose2d curPose = usesSupplier ? currentPoseSupplier.get() : currentPose;
        boolean satX = BreakerMath.isRoughlyEqualTo(triggerPose.getX(), curPose.getX(), Math.abs(tolerences.getX()));
        boolean satY = BreakerMath.isRoughlyEqualTo(triggerPose.getY(), curPose.getY(), Math.abs(tolerences.getY()));
        boolean satRot = BreakerMath.isRoughlyEqualTo(triggerPose.getRotation().getDegrees(), curPose.getRotation().getDegrees(), Math.abs(tolerences.getRotation().getDegrees()));
        return satX && satY && satRot;
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
