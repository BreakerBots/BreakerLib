// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.action.diffactionauto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.devices.sensors.BreakerPigeon2;
import frc.robot.BreakerLib.subsystemcores.drivetrain.differential.BreakerDiffDrive;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Robot moves forward/back to target distance */
public class AutoMoveStraight extends CommandBase {
  private int cyclecount;
  private BreakerPigeon2 imu;
  private double targetDistance;
  private double speedClamp;
  private double time;
  private AutoController auto;
  private BreakerDiffDrive driveTrain;
  private double ticksPerInch;
  /** Autonomous command used to move the robot forward or backward a specified number of inches
   * 
   * @param driveArg Drive subsystem from RobotContainer
   * @param imuArg IMU device from RobotContainer
   * @param distanceInches the distance in inches you want the robot to travel (+ is forward and - is reverse) (relative to intake as front)
   * @param speedLimit the precent of max speed you wish the robot to be caped at (0.0 to 1.0) (DO NOT make argument negative) (WARNING: 7.0 or above is EXTREAMLY FAST)
   * @param secArg the time limit (in seconds) on this particular instance of this command befor it times out and cancles (safty feature to prevent accadents)
   */
  public AutoMoveStraight(AutoController autoArg, BreakerDiffDrive driveTrainArg, BreakerPigeon2 imuArg, double distanceInches, double speedLimit, double secArg, double ticksPerInchArg) {
    time = secArg * 50; 
    auto = autoArg;
    imu = imuArg;
    driveTrain = driveTrainArg;
    targetDistance = distanceInches;
    speedClamp = speedLimit;
    ticksPerInch = ticksPerInchArg;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetDriveEncoders();
    imu.reset();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cyclecount ++;
    double curDist = BreakerMath.ticksToInches(driveTrain.getLeftDriveTicks(), ticksPerInch);
    System.out.println("Ticks: " + driveTrain.getLeftDriveTicks());
    double motorSpeed = auto.calculateMoveStraightPID(curDist, targetDistance);
    motorSpeed = MathUtil.clamp(motorSpeed, -speedClamp, speedClamp);
    motorSpeed += (motorSpeed > 0) ? auto.getMoveStraightFeedForward() : -auto.getMoveStraightFeedForward();
    double turnSpeed = -imu.getYawDegrees() * -0.04;
    driveTrain.arcadeDrive(motorSpeed, turnSpeed);
    // 1D movement back and forth

    System.out.println("Position error: " + auto.getMoveStraightError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cyclecount = 0;
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return auto.atMoveStraightSetPoint() || cyclecount >= time;
  }
}
