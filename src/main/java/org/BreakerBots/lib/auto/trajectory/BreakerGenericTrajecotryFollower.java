// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.auto.trajectory;

import java.util.ArrayList;
import java.util.List;

import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalCommand;
import edu.wpi.first.math.trajectory.Trajectory;

/** Add your docs here. */
public interface BreakerGenericTrajecotryFollower {
    public abstract Trajectory getCurrentTrajectory();

    public abstract Trajectory[] getAllTrajectorys();

    public abstract double getTotalPathTimeSeconds();

    public abstract double getCurrentPathTimeSeconds();

    public abstract boolean getPathStopsAtEnd();

    public abstract List<State> getAllStates();

    public abstract State getCurrentState();

    public abstract void attachConditionalCommands(BreakerConditionalCommand... conditionalCommands);

}

