// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.devices.sensors.BreakerPigeon2;
import frc.robot.BreakerLib.physics.Breaker3AxisForces;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystemcores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.swervemodules.BreakerMK4iSwerveModule;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.swervemodules.BreakerGenericSwerveModule;
import frc.robot.BreakerLib.util.BreakerRoboRIO;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerChannel;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.powermanagement.DevicePowerMode;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.selftest.SelfTest;

public class BreakerSwerveDrive implements BreakerGenericDrivetrain {
  private BreakerSwerveDriveConfig config;
  /** [0] = frontLeft, [1] = frontRight, [2] = backLeft, [3] = backRight */
  private SwerveModuleState[] targetModuleStates;

  private BreakerGenericSwerveModule[] swerveModules;

  private BreakerPigeon2 pigeon2;

  private SwerveDriveOdometry odometer;

  private Rotation2d fieldRelativeMovementAngleOffset = Rotation2d.fromDegrees(0);

  private BreakerMovementState2d prevMovementState = new BreakerMovementState2d();
  private BreakerMovementState2d curMovementState = new BreakerMovementState2d();
  private double prevOdometryUpdateTimestamp = 0;

  private boolean isInSlowMode;

  private String deviceName = "Swerve_Drivetrain";
  private String faults = null;
  /** Constructs a new swerve based drivetrain
   * @param config - the confiuration values for the drivetrain's charicteristics and behavor, passed in as a "BreakerSwerveDriveConfig" object
   * @param swerveModules - The four swerve drive modules that make up the drivetrain, must be passed in the same order shown below
   */
  public BreakerSwerveDrive(BreakerSwerveDriveConfig config, BreakerPigeon2 pigeon2, BreakerGenericSwerveModule... swerveModules) {
    odometer = new SwerveDriveOdometry(config.getKinematics(), Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]));
    this.config = config;
    this.swerveModules = swerveModules;
    this.pigeon2 = pigeon2;
    SelfTest.autoRegesterDevice(this);
  }

  /** sets each module to match a target module state in the order they were passed in
   * <p> NOTE: not affected by drive slow mode
   */
  public void setRawModuleStates(SwerveModuleState... targetModuleStates) {
    this.targetModuleStates = targetModuleStates;
    for (int i = 0; i <= swerveModules.length; i ++) {
      swerveModules[i].setModuleTarget(targetModuleStates[i]);
    }
  }

    /** Standard drivetrain movement command, specifies robot velocity in each axis including robot rotation (radian per second). 
   *<p> NOTE: All values are relative to the robot's orientation. 
   @param robotRelativeVelocities ChassisSpeeds object representing the robots velocities in each axis relative to its local refrence frame 
   @param useSlowMode wether or not to apply the set slow mode multiplier to the given speeds
   */
  public void move(ChassisSpeeds robotRelativeVelocities, boolean useSlowMode) {
    if (useSlowMode) {
      robotRelativeVelocities.vxMetersPerSecond *= config.getSlowModeLinearMultiplier();
      robotRelativeVelocities.vyMetersPerSecond *= config.getSlowModeLinearMultiplier();
      robotRelativeVelocities.omegaRadiansPerSecond *= config.getSlowModeTurnMultiplier();
    }
    setRawModuleStates(config.getKinematics().toSwerveModuleStates(robotRelativeVelocities));
  }

   /** Standard drivetrain movement command, specifies robot velocity in each axis including robot rotation (radian per second). 
   *<p> NOTE: All values are relative to the robot's orientation. 
   @param robotRelativeVelocities ChassisSpeeds object representing the robots velocities in each axis relative to its local refrence frame 
   */
  public void move(ChassisSpeeds robotRelativeVelocities) {
    move(robotRelativeVelocities, isInSlowMode);
  }

  
  /** Standard drivetrain movement command, specifies robot velocity in each axis including robot rotation (radian per second). 
   *<p> NOTE: All values are relative to the robot's orientation. */
  public void move(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec) {
    move(new ChassisSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec));
  }

   /** Standard drivetrain movement command, specifies robot velocity in each axis including robot rotation (radian per second). 
   *<p> NOTE: All values are relative to the robot's orientation. */
  public void move(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec, boolean useSlowMode) {
    move(new ChassisSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec), useSlowMode);
  }

  /** sets the target velocity of the robot to 0 in all axies */
  public void stop() {
    move(0, 0, 0);
  }

  /** Equivlent to the "move()" method but with speeds being passed in as a percentage of maximum represented as a decimal (1.0 to -1.0) */
  public void moveWithPrecentImput(double forwardPercent, double horizontalPercent, double turnPercent) {
    move((forwardPercent * config.getMaxForwardVel()), (horizontalPercent * config.getMaxSidewaysVel()), (turnPercent * config.getMaxAngleVel()));
  }

  /** effectivly equivlent to the "move()" mothod but with all vleocitys being passed in as movements relative to the field */
  public void moveRelativeToField(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec) {
    ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec, getOdometryPoseMeters().getRotation().plus(fieldRelativeMovementAngleOffset));
    move(robotRelSpeeds);
  }

  /** effectivly equivlent to the "move()" mothod but with all vleocitys being passed in as movements relative to the field (this version of the method is for use with a custom odometry source) */
  public void moveRelativeToField(double forwardVelMetersPerSec, double horizontalVelMetersPerSec, double radPerSec, BreakerGenericOdometer odometer) {
    ChassisSpeeds robotRelSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardVelMetersPerSec, horizontalVelMetersPerSec, radPerSec, odometer.getOdometryPoseMeters().getRotation().plus(fieldRelativeMovementAngleOffset));
    move(robotRelSpeeds);
  }

  /** effectivly equivlent to the "moveRelativeToField()" method but with speeds being passed in as a percentage of maximum represented as a decimal (1.0 to -1.0) */
  public void moveWithPrecentImputRelativeToField(double forwardPercent, double horizontalPercent, double turnPercent) {
    double fwdV = forwardPercent * config.getMaxForwardVel();
    double horzV = horizontalPercent * config.getMaxSidewaysVel();
    double thetaV = turnPercent * config.getMaxAngleVel();
    moveRelativeToField(fwdV, horzV, thetaV);
  }

  /** effectivly equivlent to the "moveRelativeToField()" method but with speeds being passed in as a percentage of maximum represented as a decimal (1.0 to -1.0) (this version of the method is for use with a custom odometry source) */
  public void moveWithPrecentImputRelativeToField(double forwardPercent, double horizontalPercent, double turnPercent, BreakerGenericOdometer odometer) {
    double fwdV = forwardPercent * config.getMaxForwardVel();
    double horzV = horizontalPercent * config.getMaxSidewaysVel();
    double thetaV = turnPercent * config.getMaxAngleVel();
    moveRelativeToField(fwdV, horzV, thetaV, odometer);
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[swerveModules.length];
    for (int i = 0; i <= swerveModules.length; i ++) {
      moduleStates[i] = swerveModules[i].getModuleState();
    }
    return moduleStates;
  }

  @Override
  public void updateOdometry() {
    odometer.updateWithTime(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), getSwerveModuleStates());
    calculateMovementState((Timer.getFPGATimestamp() - prevOdometryUpdateTimestamp) * 1000);
    prevOdometryUpdateTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return odometer.getPoseMeters();
  }

  public BreakerSwerveDriveConfig getConfig() {
      return config;
  }

  @Override
  public void runSelfTest() {
    faults = null;
    for (BreakerGenericSwerveModule module: swerveModules) {
      module.runModuleSelfCheck();
      if (module.hasFault()) {
        faults += " " + module.getDeviceName() + ": " + module.getFaults() + " ";
      }
    }
  }

  @Override
  public DeviceHealth getHealth() {
    return hasFault() ? DeviceHealth.FAULT : DeviceHealth.NOMINAL;
  }

  @Override
  public String getFaults() {
    return faults;
  }

  @Override
  public String getDeviceName() {
    return deviceName;
  }

  @Override
  public boolean hasFault() {
    return (faults != null);
  }

  @Override
  public void setDeviceName(String newName) {
    deviceName = newName;
    
  }

  @Override
  public void setDrivetrainBrakeMode(boolean isEnabled) {
    for (BreakerGenericSwerveModule module: swerveModules) {
      module.setDriveMotorBrakeMode(isEnabled);
    }
  }

  @Override
  public Object getBaseOdometer() {
    return odometer;
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
   odometer.resetPosition(newPose, Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]));
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return curMovementState;
  }

  private void calculateMovementState(double timeToLastUpdateMiliseconds) {
    ChassisSpeeds speeds = config.getKinematics().toChassisSpeeds(getSwerveModuleStates());
    curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(), getFieldRelativeChassisSpeeds(), timeToLastUpdateMiliseconds, prevMovementState);
    prevMovementState = curMovementState;
  }

  @Override
  public void setSlowMode(boolean isEnabled) {
    isInSlowMode = isEnabled;
  }

  @Override
  public boolean isInSlowMode() {
    return isInSlowMode;
  }

  public SwerveModuleState[] getTargetModuleStates() {
      return targetModuleStates;
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return config.getKinematics().toChassisSpeeds(getSwerveModuleStates());
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getOdometryPoseMeters().getRotation());
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds(BreakerGenericOdometer odometer) {
    return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), odometer.getOdometryPoseMeters().getRotation());
  }

  /** sets the angle all field relative movement is offset by (default is 0) */
  public void setFieldRelativeMovementAngleOffset(Rotation2d angleOffset) {
      fieldRelativeMovementAngleOffset = angleOffset;
  }
  
  /** returns the angle all field relative movement is offset by (default is 0) */
  public Rotation2d getFieldRelativeMovementAngleOffset() {
      return fieldRelativeMovementAngleOffset;
  }

  @Override
  public boolean isUnderAutomaticControl() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public DevicePowerMode getPowerMode() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void returnToAutomaticPowerManagement() {
    // TODO Auto-generated method stub
    
  }

}
