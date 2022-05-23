// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.BreakerLib.auto.trajectory.BreakerGenericTrajecotryFollower;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalCommand;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.util.BreakerLog;

/** Add your docs here. */
public class BreakerFollowSwerveTrajectory extends CommandBase implements BreakerGenericTrajecotryFollower {

    private SwerveControllerCommand controller;
    private BreakerFollowSwerveTrajectoryConfig config;
    private BreakerSwerveDrive drivetrain;
    private Subsystem requiredSubsystem; // Swerve drive subsystem
    private Trajectory[] trajectoriesToFollow;
    private int currentTrajectory = 0;
    private int prevTrajectory = -1;
    private double totalTimeSeconds = 0;
    private boolean commandIsFinished = false;
    private boolean stopAtEnd = false;
    private double currentTimeCycles = 0;
    private List<BreakerConditionalCommand> attachedCondtionalCommands;
    private Supplier<Rotation2d> rotationSupplier;
    private boolean usesSupplyedRotation = false;

    BreakerFollowSwerveTrajectory(BreakerFollowSwerveTrajectoryConfig config, boolean stopAtEnd,
            Subsystem requiredSubsystem, BreakerTrajectoryPath... trajectoryPaths) {
        usesSupplyedRotation = false;
        drivetrain = config.getDrivetrain();
        this.config = config;
        this.stopAtEnd = stopAtEnd;
        this.requiredSubsystem = requiredSubsystem;
        try {
            for (BreakerTrajectoryPath path: trajectoryPaths) {
                attachedCondtionalCommands.addAll(path.getAttachedConditionalCommands());
            }
        } catch (Exception e) {
            BreakerLog.logError(e.toString());
        }
        Trajectory[] arr = new Trajectory[trajectoryPaths.length];
        for (int i = 0; i <= trajectoryPaths.length; i++) {
            arr[i] = trajectoryPaths[i].getBaseTrajectory();
        }
        trajectoriesToFollow = arr;
    }
   
    BreakerFollowSwerveTrajectory(BreakerFollowSwerveTrajectoryConfig config, Supplier<Rotation2d> rotationSupplier, boolean stopAtEnd,
            Subsystem requiredSubsystem, BreakerTrajectoryPath... trajectoryPaths) {
        usesSupplyedRotation = true;
        drivetrain = config.getDrivetrain();
        this.config = config;
        this.stopAtEnd = stopAtEnd;
        this.requiredSubsystem = requiredSubsystem;
        this.rotationSupplier = rotationSupplier;
        try {
            for (BreakerTrajectoryPath path: trajectoryPaths) {
                attachedCondtionalCommands.addAll(path.getAttachedConditionalCommands());
            }
        } catch (Exception e) {
            BreakerLog.logError(e.toString());
        }
        Trajectory[] arr = new Trajectory[trajectoryPaths.length];
        for (int i = 0; i <= trajectoryPaths.length; i++) {
            arr[i] = trajectoryPaths[i].getBaseTrajectory();
        }
        trajectoriesToFollow = arr;
    }

    

    @Override
    public void initialize() {
        for (Trajectory trajectory : trajectoriesToFollow) {
            totalTimeSeconds += trajectory.getTotalTimeSeconds();
        }
        BreakerLog
                .logBreakerLibEvent("BreakerSwerveTrajectoryAuto command instance started, total cumulative path time: "
                        + totalTimeSeconds);
    }

    @Override
    public void execute() {
        currentTimeCycles++;
        checkAttachedCommands();
        if (currentTrajectory != prevTrajectory) {
            try {
                if (usesSupplyedRotation) {
                    controller = new SwerveControllerCommand(trajectoriesToFollow[currentTrajectory],
                        drivetrain::getOdometryPoseMeters,
                        drivetrain.getConfig().getKinematics(), config.getxPosPID(), config.getyPosPID(),
                        config.getThetaAngPID(), rotationSupplier, drivetrain::setRawModuleStates, requiredSubsystem);
                } else {
                    controller = new SwerveControllerCommand(trajectoriesToFollow[currentTrajectory],
                        drivetrain::getOdometryPoseMeters,
                        drivetrain.getConfig().getKinematics(), config.getxPosPID(), config.getyPosPID(),
                        config.getThetaAngPID(), drivetrain::setRawModuleStates, requiredSubsystem);
                }
                controller.schedule();
                BreakerLog.logBreakerLibEvent(
                        "BreakerSwerveTrajectoryAuto command instance has swiched to a new trajecotry (T-STATS: "
                                + trajectoriesToFollow[currentTrajectory].toString() + " )(T-NUM: " + currentTrajectory
                                + " of " + trajectoriesToFollow.length + " )");
            } catch (Exception e) {
                commandIsFinished = true;
            }
        }
        prevTrajectory = currentTrajectory;
        if (controller.isFinished()) {
            currentTrajectory++;
        }
    }

    public double getTotalTimeSeconds() {
        return totalTimeSeconds;
    }

    public double getCurrentTimeSeconds() {
        return (currentTimeCycles / 50);
    }

    @Override
    public void end(boolean interrupted) {
        if (stopAtEnd) {
            drivetrain.stop();
        }
        BreakerLog.logBreakerLibEvent("BreakerSwerveTrajectoryAuto command instance has ended");
    }

    @Override
    public boolean isFinished() {
        return commandIsFinished || (currentTrajectory > trajectoriesToFollow.length);
    }

    @Override
    public Trajectory getCurrentTrajectory() {
        return trajectoriesToFollow[currentTrajectory];
    }

    @Override
    public Trajectory[] getAllTrajectorys() {
        return trajectoriesToFollow;
    }

    @Override
    public double getTotalPathTimeSeconds() {
        return totalTimeSeconds;
    }

    @Override
    public double getCurrentPathTimeSeconds() {
        return (currentTimeCycles / 50);
    }

    @Override
    public boolean getPathStopsAtEnd() {
        return stopAtEnd;
    }

    @Override
    public List<State> getAllStates() {
        List<State> allStates = new ArrayList<State>();
        for (Trajectory sampTreject : trajectoriesToFollow) {
            for (State sampState : sampTreject.getStates()) {
                allStates.add(sampState);
            }
        }
        return allStates;
    }

    private double getOnlyCurrentPathTime() {
        double time = 0;
        for (int i = 0; i < currentTrajectory; i++) {
            time += trajectoriesToFollow[i].getTotalTimeSeconds();
        }
        return (getCurrentPathTimeSeconds() - time);
    }

    @Override
    public State getCurrentState() {
        return trajectoriesToFollow[currentTrajectory].sample(getCurrentPathTimeSeconds() - getOnlyCurrentPathTime());
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
                com.updateAutoRun();
            }
        } catch (Exception e) {
            BreakerLog.logError(e.toString());
        }
        
    }
}
