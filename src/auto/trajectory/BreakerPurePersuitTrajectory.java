// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

/** Add your docs here. */
public class BreakerPurePersuitTrajectory extends Trajectory{
    private double waypointIgnoreThreshold;
    private Translation2d[] waypoints;
    private TrajectoryConfig config;
    private long startTimeMS;
    private double lastUpdateTimeSeconds;
    private boolean hasStarted = false;
    private Trajectory currentTrajectory;
    public BreakerPurePersuitTrajectory(double waypointIgnoreThreshold, TrajectoryConfig config, Translation2d... waypoints) {
        this.waypointIgnoreThreshold = waypointIgnoreThreshold;
        this.waypoints = waypoints;
        this.config = config;
    }

    private long currentTimeMs() {
        return System.currentTimeMillis() - startTimeMS;
    }

    public double getCurrentPathTimeSeconds() {
        return ((double) currentTimeMs()) / 1000d;
    }

    private Translation2d getPoseAsTranslation(Pose2d pose) {
        return new Translation2d(pose.getX(), pose.getY());
    }

    public void updateConfig(TrajectoryConfig newConfig) {
        config = newConfig;
    }

    public Trajectory generate(Pose2d currentPosition, Pose2d newTarget) {
        List<Translation2d> regardedWaypoints = new ArrayList<Translation2d>();
        for (Translation2d wayPt: waypoints) {
            if (wayPt.getDistance(getPoseAsTranslation(currentPosition)) <= waypointIgnoreThreshold) {
                regardedWaypoints.add(wayPt);
            }
        }
        if (!hasStarted) {
            startTimeMS = System.currentTimeMillis();
            hasStarted = true;
        }
        waypoints = regardedWaypoints.toArray(new Translation2d[regardedWaypoints.size()]);
        lastUpdateTimeSeconds = getCurrentPathTimeSeconds();
        currentTrajectory = TrajectoryGenerator.generateTrajectory(currentPosition, regardedWaypoints, newTarget, config);
        return currentTrajectory;
    }

    public Trajectory generate(Pose2d currentPosition, Pose2d newTarget, Translation2d... newWaypoints) {
        List<Translation2d> waypts = new ArrayList<Translation2d>();
        for (Translation2d waypt: newWaypoints) {
        waypts.add(waypt);
        }
        if (!hasStarted) {
            startTimeMS = System.currentTimeMillis();
            hasStarted = true;
        }
        waypoints = newWaypoints;
        lastUpdateTimeSeconds = getCurrentPathTimeSeconds();
        currentTrajectory = TrajectoryGenerator.generateTrajectory(currentPosition, waypts, newTarget, config);
        return currentTrajectory;
    }

    public Translation2d[] getWaypoints() {
        return waypoints;
    }

    @Override
    public State sample(double timeSeconds) {
        return currentTrajectory.sample(timeSeconds - lastUpdateTimeSeconds);
    }

    @Override
    public double getTotalTimeSeconds() {
        return currentTrajectory.getTotalTimeSeconds() + lastUpdateTimeSeconds;
    }

    @Override
    public Pose2d getInitialPose() {
        return currentTrajectory.getInitialPose();
    }

    @Override
    public List<State> getStates() {
        return currentTrajectory.getStates();
    }

    @Override
    public Trajectory transformBy(Transform2d transform) {
        return currentTrajectory.transformBy(transform);
    }

    @Override
    public Trajectory concatenate(Trajectory other) {
        return currentTrajectory.concatenate(other);
    }


}
