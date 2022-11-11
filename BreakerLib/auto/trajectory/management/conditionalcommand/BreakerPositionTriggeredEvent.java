// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerPositionTriggeredEvent implements BreakerConditionalEvent{
    private Pose2d triggerPose;
    private Pose2d tolerences;
    private Command commandToRun;
    public BreakerPositionTriggeredEvent(Pose2d triggerPose, Pose2d triggerPoseTolerences, Command commandToRun) {
        this.commandToRun = commandToRun;
        this.triggerPose = triggerPose;
        tolerences = triggerPoseTolerences;
    }

    @Override
    public Command getBaseCommand() {
        return commandToRun;
    }

    @Override
    public boolean checkCondition(double currentTimeSeconds, Pose2d currentPose) {
        boolean satX = BreakerMath.isRoughlyEqualTo(triggerPose.getX(), currentPose.getX(), Math.abs(tolerences.getX()));
        boolean satY = BreakerMath.isRoughlyEqualTo(triggerPose.getY(), currentPose.getY(), Math.abs(tolerences.getY()));
        boolean satRot = BreakerMath.isRoughlyEqualTo(triggerPose.getRotation().getDegrees(), currentPose.getRotation().getDegrees(), Math.abs(tolerences.getRotation().getDegrees()));
        return satX && satY && satRot;
    }
}
