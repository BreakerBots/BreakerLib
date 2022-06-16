// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.swervemodules;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.util.BreakerCTREUtil;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerChannel;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.powermanagement.DevicePowerMode;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.selftest.SelfTest;

/** Add your docs here. */
public class BreakerMK4iSwerveModule implements BreakerGenericSwerveModule {
    private BreakerSwerveDriveConfig config;
    private String deviceName = "Swerve_Module_(SDS_MK4I)";
    private PIDController drivePID;
    private PIDController anglePID;
    // private SimpleMotorFeedforward driveFF;
    private WPI_TalonFX turnMotor;
    private WPI_TalonFX driveMotor;
    private DeviceHealth turnMotorHealth = DeviceHealth.NOMINAL;
    private DeviceHealth driveMotorHealth = DeviceHealth.NOMINAL;
    private DeviceHealth overallHealth = DeviceHealth.NOMINAL;
    private DeviceHealth encoderHealth = DeviceHealth.NOMINAL;
    private String faults = null;
    private WPI_CANCoder turnEncoder;
    /** constructs a new Swerve Drive Spetialties MK4I (inverted) swerve drive module, implaments the BreakerSwerveModule Interface
     * @param driveMotor - The TalonFX motor that moves the module's wheel linearly
     * @param turnMotor - The TalonFX motor that actuates module's wheel angle and changes the direction it is faceing
     * @param turnEncoder - The CTRE CANcoder magnetic encoder that the module uses to detirman wheel angle
     * @param config - The BreakerSwerveDriveConfig object that holds all constants for your drivetrain
     */
    public BreakerMK4iSwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX turnMotor, WPI_CANCoder turnEncoder, BreakerSwerveDriveConfig config) {
        this.config = config;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.turnEncoder = turnEncoder;

        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        BreakerCTREUtil.checkError(turnEncoder.configAllSettings(encoderConfig), " Failed to config swerve module turn encoder "); 

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.remoteFilter0.remoteSensorDeviceID = turnEncoder.getDeviceID();
        turnConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        turnConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        turnConfig.slot0.kP = config.getModuleAnglekP();
        turnConfig.slot0.kI = config.getModuleAnglekI();
        turnConfig.slot0.kD = config.getModuleAngleKd();
        BreakerCTREUtil.checkError(turnMotor.configAllSettings(turnConfig)," Failed to config swerve module turn motor "); 
        turnMotor.selectProfileSlot(0, 0);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveConfig.slot0.kP = config.getModuleVelkP();
        driveConfig.slot0.kI = config.getModuleVelkI();
        driveConfig.slot0.kD = config.getModuleVelKd();
        driveConfig.slot0.kF = config.getModuleVelKf();
        BreakerCTREUtil.checkError(driveMotor.configAllSettings(driveConfig), " Failed to config swerve module drive motor "); ;
        driveMotor.selectProfileSlot(0, 0);
    }
 
    @Override
    public void setModuleTarget(Rotation2d tgtAngle, double speedMetersPreSec) {
        turnMotor.set(TalonFXControlMode.Position, tgtAngle.getDegrees());
        driveMotor.set(TalonFXControlMode.Velocity, getMetersPerSecToFalconRSU(speedMetersPreSec));
    }

    @Override
    public void setModuleTarget(SwerveModuleState targetState) {
        setModuleTarget(targetState.angle, targetState.speedMetersPerSecond);
    }

    @Override
    public double getModuleAbsoluteAngle() {
        return turnEncoder.getAbsolutePosition();
    }

    @Override
    public double getModuleRelativeAngle() {
        return turnEncoder.getPosition();
    }
 
    @Override
    public double getModuleVelMetersPerSec() {
       return Units.inchesToMeters(BreakerMath.ticksToInches(driveMotor.getSelectedSensorVelocity() * 10,
        BreakerMath.getTicksPerInch(2048, config.getDriveMotorGearRatioToOne(), config.getWheelDiameter())));
    }

    @Override
    public double getMetersPerSecToFalconRSU(double speedMetersPerSec) {
        return (speedMetersPerSec / 10) * Units.inchesToMeters(BreakerMath.getTicksPerInch(2048, config.getDriveMotorGearRatioToOne(), config.getWheelDiameter()));
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleVelMetersPerSec(), Rotation2d.fromDegrees(getModuleRelativeAngle()));
    }

    @Override
    public boolean atAngleSetpoint() {
        return anglePID.atSetpoint();
    }

    @Override
    public boolean atVelSetpoint() {
        return drivePID.atSetpoint();
    }

    @Override
    public boolean atSetModuleState() {
        return (atAngleSetpoint() && atVelSetpoint());
    }

    @Override
    public void runModuleSelfCheck() {
        
    }

    /** returns the modules health as an array [0] = overall, [1] = drive motor, [2] = turn motor, [3] = CANcoder
     */
    @Override
    public DeviceHealth[] getModuleHealths() {
        DeviceHealth[] healths = new DeviceHealth[4];
        healths[0] = overallHealth;
        healths[1] = driveMotorHealth;
        healths[2] = turnMotorHealth;
        healths[3] = encoderHealth;
        return healths;
    }

    @Override
    public void setDriveMotorBrakeMode(boolean isEnabled) {
        driveMotor.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
        
    }

    @Override
    public void setTurnMotorBreakMode(boolean isEnabled) {
        turnMotor.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void runSelfTest() {
        faults = null;
        Faults curTurnFaults = new Faults();
        Faults curDriveFaults = new Faults();
        CANCoderFaults curEncoderFaults = new CANCoderFaults();
        turnMotor.getFaults(curTurnFaults);
        driveMotor.getFaults(curDriveFaults);
        turnEncoder.getFaults(curEncoderFaults);
        if (curDriveFaults.HardwareFailure) {
            driveMotorHealth = DeviceHealth.INOPERABLE;
            faults += " DRIVE_MOTOR_FAIL ";
        }
        if (curTurnFaults.HardwareFailure) {
            turnMotorHealth = DeviceHealth.INOPERABLE;
            faults += " TURN_MOTOR_FAIL ";
        }
        if (curTurnFaults.HardwareFailure ^ curDriveFaults.HardwareFailure) {
            overallHealth = (overallHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : overallHealth;
        } else if (curTurnFaults.HardwareFailure && curDriveFaults.HardwareFailure) {
            overallHealth = DeviceHealth.INOPERABLE;
        }
        if (curTurnFaults.SupplyUnstable) {
            faults += " TURN_MOTOR_UNSTABLE_SUPPLY ";
            turnMotorHealth = (turnMotorHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : turnMotorHealth;
        }
        if (curDriveFaults.SupplyUnstable) {
            faults += " DRIVE_MOTOR_UNSTABLE_SUPPLY ";
            driveMotorHealth = (driveMotorHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : driveMotorHealth;
        }
        if (curTurnFaults.UnderVoltage) {
            faults += " TURN_MOTOR_UNDER_6.5V ";
            turnMotorHealth = (turnMotorHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : turnMotorHealth;
        }
        if (curDriveFaults.UnderVoltage) {
            faults += " DRIVE_MOTOR_UNDER_6.5V ";
            driveMotorHealth = (driveMotorHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : driveMotorHealth;
        }
        if (curTurnFaults.SensorOutOfPhase) {
            faults += " TURN_SENSOR_OUT_OF_PHASE ";
            turnMotorHealth = (turnMotorHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : turnMotorHealth;
        }
        if (curDriveFaults.SensorOutOfPhase) {
            faults += " DRIVE_SENSOR_OUT_OF_PHASE ";
            driveMotorHealth = (driveMotorHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : driveMotorHealth;
        }
        if (curEncoderFaults.HardwareFault) {
            faults += " ABSOLUTE_ENCODER_FAIL ";
            overallHealth = (overallHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : overallHealth;
            encoderHealth = DeviceHealth.INOPERABLE;
        }
        if (curEncoderFaults.MagnetTooWeak) {
            faults += " ABSOLUET_ENCODER_WEAK_MAG ";
            encoderHealth = (encoderHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : encoderHealth;
            overallHealth = (overallHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : overallHealth;
        }
        if (!curDriveFaults.HardwareFailure && !curTurnFaults.HardwareFailure && !curTurnFaults.SupplyUnstable && !curDriveFaults.SupplyUnstable && !curEncoderFaults.MagnetTooWeak && !curEncoderFaults.HardwareFault) {
            faults = null;
            driveMotorHealth = DeviceHealth.NOMINAL;
            turnMotorHealth = DeviceHealth.NOMINAL;
            overallHealth = DeviceHealth.NOMINAL;
        }
    }

    @Override
    public DeviceHealth getHealth() {
        return overallHealth;
    }

    @Override
    public String getFaults() {
        return faults;
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }

    /** returns the device's overall health */
    @Override
    public boolean hasFault() {
        return overallHealth != DeviceHealth.NOMINAL;
    }

    @Override
    public void setDeviceName(String newName) {
        deviceName = newName;
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
