// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.differential;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.devices.sensors.BreakerPigeon2;
import frc.robot.BreakerLib.driverstation.BreakerFieldWidget;
import frc.robot.BreakerLib.physics.Breaker3AxisForces;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.differential.BreakerDiffDriveState;
import frc.robot.BreakerLib.subsystemcores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.util.BreakerCTREUtil;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerChannel;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.powermanagement.DevicePowerMode;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.selftest.SelfTest;

public class BreakerDiffDrive implements BreakerGenericDrivetrain {
  private WPI_TalonFX leftLead;
  private WPI_TalonFX[] leftMotors;
  private MotorControllerGroup leftDrive;

  private WPI_TalonFX rightLead;
  private WPI_TalonFX[] rightMotors;
  private MotorControllerGroup rightDrive;

  private DifferentialDrive diffDrive;
  private BreakerDiffDriveConfig driveConfig;

  private BreakerPigeon2 pigeon2;
  private DifferentialDriveOdometry driveOdometer;
  private BreakerMovementState2d prevMovementState = new BreakerMovementState2d();
  private BreakerMovementState2d curMovementState = new BreakerMovementState2d();
  private double prevOdometryUpdateTimestamp = 0;

  private String deviceName = "Differential_Drivetrain";
  private String faults = null;
  private boolean hasFault = false;

  private boolean isInSlowMode;
  private boolean invertL;
  private boolean invertR;

  private boolean isAutoPowerManaged = true;
  private DevicePowerMode powerMode = DevicePowerMode.FULL_POWER_MODE;
  
  /** Creates a new West Coast Drive. */
  public BreakerDiffDrive(WPI_TalonFX[] leftMotors, WPI_TalonFX[] rightMotors, boolean invertL, boolean invertR, BreakerPigeon2 pigeon2, BreakerDiffDriveConfig driveConfig) {
    this.leftMotors = leftMotors;
    this.invertL = invertL;
    leftLead = leftMotors[0];
    leftDrive = new MotorControllerGroup(leftMotors);
    leftDrive.setInverted(invertL);

    this.rightMotors = rightMotors;
    this.invertR = invertR;
    rightLead = rightMotors[0];
    rightDrive = new MotorControllerGroup(rightMotors);
    rightDrive.setInverted(invertR);

    diffDrive = new DifferentialDrive(leftDrive, rightDrive);

    driveOdometer = new DifferentialDriveOdometry(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]));

    this.driveConfig = driveConfig;
    this.pigeon2 = pigeon2;
    SelfTest.autoRegesterDevice(this);
  }

  /** Standard drive command, is affected by slow mode */
  public void arcadeDrive(double netSpeed, double turnSpeed) {
    if (isInSlowMode) {
      netSpeed *= driveConfig.getSlowModeForwardMultiplier();
      turnSpeed *= driveConfig.getSlowModeTurnMultiplier();
    }

    diffDrive.arcadeDrive(netSpeed, turnSpeed);
  }

  /** Standard drive command, optionaly applys slow mode */
  public void arcadeDrive(double netSpeed, double turnSpeed, boolean useSlowMode) {
    if (useSlowMode) {
      netSpeed *= driveConfig.getSlowModeForwardMultiplier();
      turnSpeed *= driveConfig.getSlowModeTurnMultiplier();
    }

    diffDrive.arcadeDrive(netSpeed, turnSpeed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankMoveVoltage(double leftVoltage, double rightVoltage) {
    leftDrive.setVoltage(leftVoltage);
    rightDrive.setVoltage(rightVoltage);
    diffDrive.feed();
    System.out.println("LV: " + leftVoltage + " RV: " + rightVoltage);
  }

  public void resetDriveEncoders() {
    leftLead.setSelectedSensorPosition(0);
    rightLead.setSelectedSensorPosition(0);
  }

  public double getLeftDriveTicks() {
    return invertL ? -leftLead.getSelectedSensorPosition() : leftLead.getSelectedSensorPosition();
  }

  public double getLeftDriveInches() {
    return (getLeftDriveTicks() / driveConfig.getTicksPerInch());
  }

  public double getLeftDriveMeters() {
    return Units.inchesToMeters(getLeftDriveInches());
  }

  public double getLeftDriveVelocityRSU() {
    return invertL ? -leftLead.getSelectedSensorVelocity() : leftLead.getSelectedSensorVelocity();
  }

  public double getRightDriveTicks() {
    return invertR ? -rightLead.getSelectedSensorPosition() : rightLead.getSelectedSensorPosition();
  }

  public double getRightDriveInches() {
    return getRightDriveTicks() / driveConfig.getTicksPerInch();
  }

  public double getRightDriveMeters() {
    return Units.inchesToMeters(getRightDriveInches());
  }

  public double getRightDriveVelocityRSU() {
    return invertR ? -rightLead.getSelectedSensorVelocity() : rightLead.getSelectedSensorVelocity();
  }

  public void setDrivetrainBrakeMode(boolean isEnabled) {
    BreakerCTREUtil.setBrakeMode(isEnabled, leftMotors);
    BreakerCTREUtil.setBrakeMode(isEnabled, rightMotors);
  }
  
  /** Returns an instance of the drivetrain's left side lead motor */
  public WPI_TalonFX getLeftLeadMotor() {
    return leftLead;
  }

  /** Returns an instance of the drivetrain's right side lead motor */
  public WPI_TalonFX getRightLeadMotor() {
    return rightLead;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return new SimpleMotorFeedforward(driveConfig.getFeedForwardKs(), driveConfig.getFeedForwardKv(), driveConfig.getFeedForwardKa());
  }

  public DifferentialDriveKinematics getKinematics() {
    return driveConfig.getKinematics();
  }

  public PIDController getLeftPIDController() {
    return driveConfig.getLeftPID();
  }

  public PIDController getRightPIDController() {
    return driveConfig.getRightPID();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds( BreakerUnits.inchesToMeters((getLeftDriveVelocityRSU() / driveConfig.getTicksPerInch()) * 10),
     BreakerUnits.inchesToMeters((getRightDriveVelocityRSU() / driveConfig.getTicksPerInch()) * 10));
  }

  @Override
  public void setSlowMode(boolean isEnabled) {
    isInSlowMode = isEnabled;
  }

  @Override
  public boolean isInSlowMode() {
    return isInSlowMode;
  }

  public BreakerDiffDriveState getDiffDriveState() {
    return new BreakerDiffDriveState(getWheelSpeeds(), getLeftDriveMeters(), getRightDriveMeters());
  }

  @Override
  public void updateOdometry() {
    driveOdometer.update(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), getLeftDriveMeters(), getRightDriveMeters());
    calculateMovementState((Timer.getFPGATimestamp() - prevOdometryUpdateTimestamp) * 1000);
    prevOdometryUpdateTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public Pose2d getOdometryPoseMeters() {
    return driveOdometer.getPoseMeters();
  }

  @Override
  public void runSelfTest() {
    faults = null;
    hasFault = false;
    StringBuilder work = new StringBuilder();
    for (WPI_TalonFX motorL: leftMotors) {
      Faults motorFaults = new Faults();
      motorL.getFaults(motorFaults);
      if (motorFaults.hasAnyFault()) {
        hasFault = true;
        work.append(" MOTOR ID (" + motorL.getDeviceID() + ") FAULTS: ");
        work.append(BreakerCTREUtil.getMotorFaultsAsString(motorFaults));
      }
    }
    for (WPI_TalonFX motorR: rightMotors) {
      Faults motorFaults = new Faults();
      motorR.getFaults(motorFaults);
      if (motorFaults.hasAnyFault()) {
        hasFault = true;
        work.append(" MOTOR ID (" + motorR.getDeviceID() + ") FAULTS: ");
        work.append(BreakerCTREUtil.getMotorFaultsAsString(motorFaults));
      }
    }
    faults = work.toString();
  }

  @Override
  public DeviceHealth getHealth() {
    return hasFault ? DeviceHealth.FAULT : DeviceHealth.NOMINAL;
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
    return hasFault;
  }

  @Override
  public void setDeviceName(String newName) {
    deviceName = newName;
    
  }

  @Override
  public void setOdometryPosition(Pose2d newPose) {
    resetDriveEncoders();
    driveOdometer.resetPosition(newPose, Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]));
  }

  @Override
  public Object getBaseOdometer() {
    return driveOdometer;
  }

  @Override
  public BreakerMovementState2d getMovementState() {
    return curMovementState;
  }

  private void calculateMovementState(double timeToLastUpdateMilisecods) {
    BreakerMovementState2d curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(), getFieldRelativeChassisSpeeds(), timeToLastUpdateMilisecods, prevMovementState);
    prevMovementState = curMovementState;
  }

  public WPI_TalonFX[] getLeftMotors() {
      return leftMotors;
  }

  public WPI_TalonFX[] getRightMotors() {
      return rightMotors;
  }

  @Override
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return driveConfig.getKinematics().toChassisSpeeds(getWheelSpeeds());
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getOdometryPoseMeters().getRotation());
  }

  @Override
  public ChassisSpeeds getFieldRelativeChassisSpeeds(BreakerGenericOdometer odometer) {
    return BreakerMath.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), odometer.getOdometryPoseMeters().getRotation());
  }

  @Override
  public boolean isUnderAutomaticControl() {
    return isAutoPowerManaged;
  }

  @Override
  public DevicePowerMode getPowerMode() {
    return powerMode;
  }

  @Override
  public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig) {
      
    return null;
  }

  @Override
  public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
    isAutoPowerManaged = false;
    
  }

  @Override
  public void returnToAutomaticPowerManagement() {
    isAutoPowerManaged = true;
  }
}
