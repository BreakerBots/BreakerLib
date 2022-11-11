// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.modules;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerCTREUtil;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerREVUtil;

/** Swerve Drive Specialties' MK4i swerve module driven by Spark Max motor controllers with Neo motors. */
public class BreakerMK4iNeoSwerveModule implements BreakerGenericSwerveModule {

    private BreakerArbitraryFeedforwardProvider ffProvider;
    private BreakerSwerveDriveConfig config;
    private String faults = null, deviceName = "Swerve_Module_(SDS_MK4I)";
    private CANSparkMax turnMotor, driveMotor;
    private PIDController turnPID;
    private WPI_CANCoder turnEncoder;
    private SwerveModuleState targetModuleState;
    private DeviceHealth turnMotorHealth = DeviceHealth.NOMINAL, driveMotorHealth = DeviceHealth.NOMINAL,
            overallHealth = DeviceHealth.NOMINAL, encoderHealth = DeviceHealth.NOMINAL;

    /**
     * Constructs a new Swerve Drive Spetialties MK4i (inverted) swerve drive
     * module, implements the BreakerSwerveModule interface.
     * 
     * @param driveMotor  - The SparkMax motor controller conected to a NEO motor
     *                    that moves the module's wheel linearly.
     * @param turnMotor   - The SparkMax motor controller conected to a NEO motor
     *                    that actuates module's wheel angle and
     *                    changes the direction it is facing.
     * @param turnEncoder - The CTRE CANCoder magnetic encoder that the module uses
     *                    to determine wheel angle.
     * @param config      - The BreakerSwerveDriveConfig object that holds all
     *                    constants for your drivetrain
     */
    public BreakerMK4iNeoSwerveModule(CANSparkMax driveMotor, CANSparkMax turnMotor, WPI_CANCoder turnEncoder,
            BreakerSwerveDriveConfig config, double encoderAbsoluteAngleOffsetDegrees, boolean invertDriveOutput,
            boolean invertTurnOutput) {
        this.config = config;
        this.turnMotor = turnMotor;
        this.driveMotor = driveMotor;
        this.turnEncoder = turnEncoder;

        turnPID = new PIDController(config.getModuleAnglekP(), config.getModuleAnglekI(), config.getModuleAngleKd());
        turnPID.enableContinuousInput(-180.0, 180.0);

        BreakerCANCoderFactory.configExistingCANCoder(turnEncoder, SensorInitializationStrategy.BootToAbsolutePosition,
                AbsoluteSensorRange.Signed_PlusMinus180, encoderAbsoluteAngleOffsetDegrees, false);

        turnMotor.setInverted(invertTurnOutput);
        turnMotor.setSmartCurrentLimit(80);

        SparkMaxPIDController drivePID = driveMotor.getPIDController();
        drivePID.setP(config.getModuleVelkP());
        drivePID.setI(config.getModuleVelkI());
        drivePID.setD(config.getModuleVelKd());
        drivePID.setFF(config.getModuleVelKf());

        driveMotor.setSmartCurrentLimit(80);
        driveMotor.setInverted(invertDriveOutput);

        ffProvider = config.getArbitraryFeedforwardProvider();

        targetModuleState = new SwerveModuleState();
    }

    @Override
    public void setModuleTarget(Rotation2d tgtAngle, double speedMetersPerSec) {
        SmartDashboard.putNumber(deviceName + " ANGLE IN", tgtAngle.getDegrees());
        SmartDashboard.putNumber(deviceName + " SPEED IN", speedMetersPerSec);

        turnMotor.set(turnPID.calculate(turnEncoder.getAbsolutePosition(), tgtAngle.getDegrees()));
        driveMotor.getPIDController().setReference(getMetersPerSecToNativeVelUnits(speedMetersPerSec),
                CANSparkMax.ControlType.kVelocity, 0, ffProvider.getArbitraryFeedforwardValue(speedMetersPerSec),
                ArbFFUnits.kPercentOut);

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
        return Units
                .inchesToMeters((driveMotor.getEncoder().getVelocity() * (config.getWheelDiameter() * Math.PI)) / 60.0);
    }

    @Override
    public double getMetersPerSecToNativeVelUnits(double speedMetersPerSec) {
        return Units.metersToInches(speedMetersPerSec * 60.0) / (config.getWheelDiameter() * Math.PI);
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
        driveMotor.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);

    }

    @Override
    public void setTurnMotorBrakeMode(boolean isEnabled) {
        turnMotor.setIdleMode(isEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setModuleBrakeMode(boolean isEnabled) {
        setDriveMotorBrakeMode(isEnabled);
        setTurnMotorBrakeMode(isEnabled);
    }

    @Override
    public void runSelfTest() {
        faults = null;
        turnMotorHealth = DeviceHealth.NOMINAL;
        driveMotorHealth = DeviceHealth.NOMINAL;
        overallHealth = DeviceHealth.NOMINAL;
        encoderHealth = DeviceHealth.NOMINAL;
        if (driveMotor.getFaults() != 0) {
            Pair<DeviceHealth, String> driveFs = BreakerREVUtil.getSparkMaxHealthAndFaults(driveMotor.getFaults());
            faults += " Drive_Motor(" + driveFs.getSecond() + ")";
            driveMotorHealth = driveFs.getFirst();
        }
        if (turnMotor.getFaults() != 0) {
            Pair<DeviceHealth, String> turnFs = BreakerREVUtil.getSparkMaxHealthAndFaults(turnMotor.getFaults());
            faults += " Turn_Motor(" + turnFs.getSecond() + ")";
            turnMotorHealth = turnFs.getFirst();
        }
        CANCoderFaults encoderFaults = new CANCoderFaults();
        turnEncoder.getFaults(encoderFaults);
        if (encoderFaults.hasAnyFault()) {
            Pair<DeviceHealth, String> encodeFs = BreakerCTREUtil.getCANCoderHealthAndFaults(encoderFaults);
            faults += " Turn_Encoder(" + encodeFs.getSecond() + ")";
            encoderHealth = encodeFs.getFirst();
        }

        if (driveMotorHealth != DeviceHealth.NOMINAL || turnMotorHealth != DeviceHealth.NOMINAL
                || encoderHealth != DeviceHealth.NOMINAL) {
            overallHealth = DeviceHealth.INOPERABLE;
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
        return BreakerGenericSwerveModule.getModuleAsString("SDS_MK4I(Neo)", this);
    }
}
