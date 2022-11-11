// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.diff;

import java.util.ArrayList;
import java.util.Iterator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerTrajectoryPath;
import frc.robot.BreakerLib.auto.trajectory.management.conditionalcommand.BreakerConditionalEvent;

public class BreakerRamsete extends CommandBase {
  /** Creates a new BreakerRamsete. */
  private Timer timer;
  private RamseteCommand ramsete;
  private BreakerRamseteConfig config;
  private BreakerTrajectoryPath trajectoryPath;
  private ArrayList<BreakerConditionalEvent> remainingEvents;
  public BreakerRamsete(BreakerRamseteConfig config, BreakerTrajectoryPath trajectoryPath) {
    this.trajectoryPath = trajectoryPath;
    this.config = config;
    ramsete = new RamseteCommand(trajectoryPath.getBaseTrajectory(), config.getOdometer()::getOdometryPoseMeters, 
      config.getRamseteController(), config.getDrivetrain().getConfig().getFeedForward(), config.getDrivetrain().getConfig().getKinematics(), 
      config.getDrivetrain()::getWheelSpeeds, config.getDrivetrain().getConfig().getLeftPID(), config.getDrivetrain().getConfig().getRightPID(), 
      config.getDrivetrain()::tankDriveVoltage, config.getDrivetrain());
    ramsete.schedule();
    remainingEvents = new ArrayList<>(trajectoryPath.getAttachedConditionalEvents());
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (remainingEvents.size() > 0) {
      Iterator<BreakerConditionalEvent> iterator = remainingEvents.iterator();
      while (iterator.hasNext()) {
        BreakerConditionalEvent ev = iterator.next();
        if (ev.checkCondition(timer.get(), config.getOdometer().getOdometryPoseMeters())) {
          ev.getBaseCommand().schedule();;
          iterator.remove();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      if (trajectoryPath.stopAtEnd()) {
        config.getDrivetrain().stop();
      }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramsete.isFinished();
  }
}
