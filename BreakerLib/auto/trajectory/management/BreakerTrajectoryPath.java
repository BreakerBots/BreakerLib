// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalEvent;

/** Represents a trajectory and its optionaly attached {@link BreakerConditionalEvent} instnaces */
public class BreakerTrajectoryPath {
    private List<BreakerConditionalEvent> attachedConditionalEvents = new ArrayList<>();
    private Trajectory baseTrajectory;
    private boolean stopAtEnd;

    /**Creates a new {@link BreakerTrajectoryPath} that stops the robot when finished and has 
     * no attached {@link BreakerConditionalEvent} instances  
     * @param baseTrajectory The {@link Trajectory} this path represnets
     */
    public BreakerTrajectoryPath(Trajectory baseTrajectory) {
        this(baseTrajectory, true);
    }

    /**Creates a new {@link BreakerTrajectoryPath} that stops the robot when finished
     * @param baseTrajectory The {@link Trajectory} this path represnets
     * @param conditionalEvents An array of {@link BreakerConditionalEvent} instances attached to this path
     */
    public BreakerTrajectoryPath(Trajectory baseTrajectory, BreakerConditionalEvent... conditionalEvents) {
        this(baseTrajectory, true, conditionalEvents);
    }

    /**Creates a new {@link BreakerTrajectoryPath} that has no attached {@link BreakerConditionalEvent} instances  
     * @param baseTrajectory The {@link Trajectory} this path represnets
     * @param stopAtEnd A boolean represeting wheather or not this path stops the robot when finished
     */
    public BreakerTrajectoryPath(Trajectory baseTrajectory, boolean stopAtEnd) {
        this.baseTrajectory = baseTrajectory;
        this.stopAtEnd = stopAtEnd;
        attachedConditionalEvents= new ArrayList<>();
    }

    /**Creates a new {@link BreakerTrajectoryPath}
     * @param baseTrajectory The {@link Trajectory} this path represnets
     * @param stopAtEnd A boolean represeting wheather or not this path stops the robot when finished
     * @param conditionalEvents An array of {@link BreakerConditionalEvent} instances attached to this path
     */
    public BreakerTrajectoryPath(Trajectory baseTrajectory, boolean stopAtEnd, BreakerConditionalEvent... conditionalEvents) {
        this.baseTrajectory = baseTrajectory;
        this.stopAtEnd = stopAtEnd;
        for (BreakerConditionalEvent com: conditionalEvents) {
            attachedConditionalEvents.add(com);
        }
    }

    public List<BreakerConditionalEvent> getAttachedConditionalEvents() {
        return attachedConditionalEvents;
    }

    public Trajectory getBaseTrajectory() {
        return baseTrajectory;
    }

    public boolean stopAtEnd() {
        return stopAtEnd;
    }
}
