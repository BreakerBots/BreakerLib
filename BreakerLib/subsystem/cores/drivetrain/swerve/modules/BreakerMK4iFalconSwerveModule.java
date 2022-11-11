// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerCTREUtil;

/** Swerve Drive Specialties' MK4i swerve module driven by Falcon 500 motors. */
public class BreakerMK4iFalconSwerveModule implements BreakerGenericSwerveModule {

    private BreakerArbitraryFeedforwardProvider ffProvider;
    private BreakerSwerveDriveConfig config;
    private String faults = null, deviceName = "Swerve_Module_(SDS_MK4I)";
    private WPI_TalonFX turnMotor, driveMotor;
    private WPI_CANCoder turnEncoder;
    private SwerveModuleState targetModuleState;
    private DeviceHealth turnMotorHealth = DeviceHealth.NOMINAL, driveMotorHealth = DeviceHealth.NOMINAL,
            overallHealth = DeviceHealth.NOMINAL, encoderHealth = DeviceHealth.NOMINAL;

    /**
     * Constructs a new Swerve Drive Specialties MK4i (inverted) swerve drive
     * module, implements the {@link BreakerSwerveModule} interface
     * 
     * @param driveMotor  - The TalonFX motor that moves the module's wheel linearly.
     * @param turnMotor   - The TalonFX motor that actuates module's wheel angle and
     *                    changes the direction it is facing.
     * @param turnEncoder - The CTRE CANcoder magnetic encoder that the module uses
     *                    to determine wheel angle.
     * @param config      - The BreakerSwerveDriveConfig object that holds all
     *                    constants for your drivetrain
     */
    public BreakerMK4iFalconSwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX turnMotor, WPI_CANCoder turnEncoder,
            BreakerSwerveDriveConfig config, double encoderAbsoluteAngleOffsetDegrees, boolean invertDriveOutput, boolean invertTurnOutput) {
        this.config = config;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.turnEncoder = turnEncoder;

        BreakerCANCoderFactory.configExistingCANCoder(turnEncoder, SensorInitializationStrategy.BootToAbsolutePosition, 
            AbsoluteSensorRange.Signed_PlusMinus180, encoderAbsoluteAngleOffsetDegrees, false);

        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.remoteFilter0.remoteSensorDeviceID = turnEncoder.getDeviceID();
        turnConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        turnConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        turnConfig.slot0.kP = config.getModuleAnglekP();
        turnConfig.slot0.kI = config.getModuleAnglekI();
        turnConfig.slot0.kD = config.getModuleAngleKd();
        turnConfig.slot0.closedLoopPeakOutput = 1.0;
        turnConfig.peakOutputForward = 1.0;
        turnConfig.peakOutputReverse = -1.0;
        turnConfig.voltageCompSaturation = 12.0;
        turnConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
        BreakerCTREUtil.checkError(turnMotor.configAllSettings(turnConfig),
                " Failed to config swerve module turn motor ");
        turnMotor.selectProfileSlot(0, 0);
        turnMotor.setSensorPhase(true);
        turnMotor.setInverted(invertTurnOutput);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.set(ControlMode.Position, 0);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        driveConfig.slot0.kP = config.getModuleVelkP();
        driveConfig.slot0.kI = config.getModuleVelkI();
        driveConfig.slot0.kD = config.getModuleVelKd();
        driveConfig.slot0.kF = config.getModuleVelKf();
        driveConfig.slot0.closedLoopPeakOutput = 1.0;
        driveConfig.peakOutputForward = 1.0;
        driveConfig.peakOutputReverse = -1.0;
        driveConfig.voltageCompSaturation = 12.0;
        driveConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
        BreakerCTREUtil.checkError(driveMotor.configAllSettings(driveConfig),
                " Failed to config swerve module drive motor ");
        ;
        driveMotor.selectProfileSlot(1, 0);
        driveMotor.setInverted(invertDriveOutput);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.set(ControlMode.Velocity, 0.0);

        ffProvider = config.getArbitraryFeedforwardProvider();

        targetModuleState = new SwerveModuleState();
    }

    @Override
    public void setModuleTarget(Rotation2d tgtAngle, double speedMetersPerSec) {
        double relTgtAng = getModuleRelativeAngle() + (tgtAngle.minus(Rotation2d.fromDegrees(getModuleAbsoluteAngle())).getDegrees()); // Aproximates continuous pid input

        turnMotor.set(TalonFXControlMode.Position, BreakerUnits.degreesToCANCoderNativeUnits(relTgtAng));
        driveMotor.set(TalonFXControlMode.Velocity, getMetersPerSecToNativeVelUnits(speedMetersPerSec),
                DemandType.ArbitraryFeedForward, ffProvider.getArbitraryFeedforwardValue(speedMetersPerSec));

        SmartDashboard.putNumber(deviceName + " ANGLE IN", tgtAngle.getDegrees());
        SmartDashboard.putNumber(deviceName +" SPEED IN", speedMetersPerSec);
        SmartDashboard.putNumber(deviceName + "ANGLE OUT", getModuleState().angle.getDegrees());
        SmartDashboard.putNumber(deviceName + "SPEED OUT", getModuleState().speedMetersPerSecond);

        targetModuleState = new SwerveModuleState(speedMetersPerSec, tgtAngle);
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
    public double getMetersPerSecToNativeVelUnits(double speedMetersPerSec) {
        return (speedMetersPerSec / 10) * Units.inchesToMeters(
                BreakerMath.getTicksPerInch(2048, config.getDriveMotorGearRatioToOne(), config.getWheelDiameter()));
    }

    @Override
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getModuleVelMetersPerSec(), Rotation2d.fromDegrees(getModuleAbsoluteAngle()));
    }

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
    public void setTurnMotorBrakeMode(boolean isEnabled) {
        turnMotor.setNeutralMode(isEnabled ? NeutralMode.Brake : NeutralMode.Coast);
    }

    @Override
    public void setModuleBrakeMode(boolean isEnabled) {
        setDriveMotorBrakeMode(isEnabled);
        setTurnMotorBrakeMode(isEnabled);
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
            faults += " ANGLE_ENCODER_FAIL ";
            overallHealth = (overallHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : overallHealth;
            encoderHealth = DeviceHealth.INOPERABLE;
        }
        if (curEncoderFaults.MagnetTooWeak) {
            faults += " ANGLE_ENCODER_WEAK_MAG ";
            encoderHealth = (encoderHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : encoderHealth;
            overallHealth = (overallHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : overallHealth;
        }
        if (!curDriveFaults.HardwareFailure && !curTurnFaults.HardwareFailure && !curTurnFaults.SupplyUnstable
                && !curDriveFaults.SupplyUnstable && !curEncoderFaults.MagnetTooWeak
                && !curEncoderFaults.HardwareFault) {
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

    /** @return The device's overall health. */
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

    @Override
    public SwerveModuleState getModuleTargetState() {
        return targetModuleState;
    }

    @Override
    public String toString() {
        return BreakerGenericSwerveModule.getModuleAsString("SDS_MK4I(Falcon)", this);
    }
}
