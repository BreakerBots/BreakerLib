// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalCommand;

/** Add your docs here. */
public class BreakerTrajectoryPath {
    private List<BreakerConditionalCommand> attachedConditionalCommands = new ArrayList<>();
    private Trajectory baseTrajectory;
    public BreakerTrajectoryPath(Trajectory baseTrajectory, BreakerConditionalCommand... conditionalCommands) {
        this.baseTrajectory = baseTrajectory;
        for (BreakerConditionalCommand com: conditionalCommands) {
            attachedConditionalCommands.add(com);
        }
    }

    public BreakerTrajectoryPath(Trajectory baseTrajectory) {
        this.baseTrajectory = baseTrajectory;
    }

    public List<BreakerConditionalCommand> getAttachedConditionalCommands() {
        return attachedConditionalCommands;
    }

    public Trajectory getBaseTrajectory() {
        return baseTrajectory;
    }
}
