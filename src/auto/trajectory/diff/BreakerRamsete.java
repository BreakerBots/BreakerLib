// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.diff;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.BreakerLib.auto.trajectory.BreakerGenericTrajecotryFollower;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalCommand;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerDiffDrive;
import frc.robot.BreakerLib.util.BreakerLog;

/** OUR version of a differential drive ramsete command.  */
public class BreakerRamsete extends CommandBase implements BreakerGenericTrajecotryFollower {
    private RamseteCommand ramsete;
    private RamseteController ramseteController;
    private BreakerDiffDrive drivetrain;
    private double currentTimestamp = Timer.getFPGATimestamp();
    private final double startTimestamp = Timer.getFPGATimestamp();
    private double currentTimeSeconds = 0.0;
    private double totalTimeSeconds = 0;
    private boolean stopAtEnd;
    private Trajectory trajectoryToFollow;
    private List<BreakerConditionalCommand> attachedCondtionalCommands;

    /**
     * Constructor for BreakerRamsete controller.
     * 
     * @param trajectoryToFollow Self-explanatory.
     * @param drivetrain Differential drivetrain to use.
     * @param subsystemRequirements Self-explanatory.
     * @param ramseteB Proportional constant for ramsete.
     * @param ramseteZeta Damping constant for ramsete.
     * @param stopAtEnd Whether the robot stops on completion.
     */
    public BreakerRamsete(BreakerTrajectoryPath trajectoryPath, BreakerDiffDrive drivetrain,
            Subsystem subsystemRequirements, double ramseteB,
            double ramseteZeta, boolean stopAtEnd) {
        this.drivetrain = drivetrain;
        trajectoryToFollow = trajectoryPath.getBaseTrajectory();

        BreakerLog.logBreakerLibEvent("BreakerRamsete command instance has started, total cumulative path time: "
                + trajectoryToFollow.getTotalTimeSeconds());

        ramseteController = new RamseteController(ramseteB, ramseteZeta);
        ramsete = new RamseteCommand(trajectoryToFollow, drivetrain::getOdometryPoseMeters, ramseteController,
                drivetrain.getFeedforward(),
                drivetrain.getKinematics(), drivetrain::getWheelSpeeds, drivetrain.getLeftPIDController(),
                drivetrain.getRightPIDController(), drivetrain::tankMoveVoltage, subsystemRequirements);
        ramsete.schedule();

        totalTimeSeconds = trajectoryToFollow.getTotalTimeSeconds();
        this.stopAtEnd = stopAtEnd;

        try {
            attachedCondtionalCommands.addAll(trajectoryPath.getAttachedConditionalCommands());
        } catch (Exception e) {
            BreakerLog.logError(e.toString());
        }
    }

    public BreakerRamsete(BreakerTrajectoryPath trajectoryPath, BreakerDiffDrive drivetrain,
            Supplier<Pose2d> currentPoseSupplyer, Subsystem subsystemRequirements, 
            double ramseteB, double ramseteZeta,  boolean stopAtEnd) {
        BreakerLog.logBreakerLibEvent("BreakerRamsete command instance has started, total cumulative path time: "
                + trajectoryToFollow.getTotalTimeSeconds());
        this.drivetrain = drivetrain;
        this.trajectoryToFollow = trajectoryPath.getBaseTrajectory();

        ramseteController = new RamseteController(ramseteB, ramseteZeta);
        ramsete = new RamseteCommand(trajectoryToFollow, currentPoseSupplyer, ramseteController,
                drivetrain.getFeedforward(),
                drivetrain.getKinematics(), drivetrain::getWheelSpeeds, drivetrain.getLeftPIDController(),
                drivetrain.getRightPIDController(), drivetrain::tankMoveVoltage, subsystemRequirements);
        ramsete.schedule();

        totalTimeSeconds = trajectoryToFollow.getTotalTimeSeconds();
        this.stopAtEnd = stopAtEnd;

        try {
            attachedCondtionalCommands.addAll(trajectoryPath.getAttachedConditionalCommands());
        } catch (Exception e) {
            BreakerLog.logError(e.toString());
        }
    }

    @Override
    public void execute() {
        calculateTime();
        checkAttachedCommands();
    }

    @Override
    public void end(boolean interrupted) {
        if (stopAtEnd) {
            drivetrain.arcadeDrive(0, 0);
        }
        BreakerLog.logBreakerLibEvent("BreakerRamsete command instance has ended");
    }

    @Override
    public boolean isFinished() {
        return ramsete.isFinished();
    }

    @Override
    public Trajectory getCurrentTrajectory() {
        return trajectoryToFollow;
    }

    @Override
    public Trajectory[] getAllTrajectorys() {
        Trajectory[] traject = new Trajectory[1];
        traject[0] = trajectoryToFollow;
        return traject;
    }

    @Override
    public double getTotalPathTimeSeconds() {
        return totalTimeSeconds;
    }

    @Override
    public double getCurrentPathTimeSeconds() {
        return currentTimeSeconds;
    }

    @Override
    public boolean getPathStopsAtEnd() {
        return stopAtEnd;
    }

    @Override
    public List<State> getAllStates() {
        return trajectoryToFollow.getStates();
    }

    @Override
    public State getCurrentState() {
        return trajectoryToFollow.sample(getCurrentPathTimeSeconds());
    }

    @Override
    public void attachConditionalCommands(BreakerConditionalCommand... conditionalCommands) {
        for (BreakerConditionalCommand com: conditionalCommands) {
            attachedCondtionalCommands.add(com);
        }
    }

    private void checkAttachedCommands() {
        try {
            for (BreakerConditionalCommand com: attachedCondtionalCommands) {
                com.updateAutoRun(currentTimeSeconds, drivetrain.getOdometryPoseMeters());
            }
        } catch (Exception e) {}
        
    }

    private void calculateTime() {
        currentTimestamp = Timer.getFPGATimestamp();
        currentTimeSeconds = startTimestamp - currentTimestamp;
    }

}
