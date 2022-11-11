// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.devices.BreakerGenericLoopedDevice;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;
import frc.robot.BreakerLib.util.BreakerTriplet;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.suites.BreakerGenericTestSuiteImplementation;
import frc.robot.BreakerLib.util.test.suites.flywheelSuite.BreakerFlywheelTestSuite;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerCTREUtil;

/** A class representing a robot's shooter flywheel and its assocated controle loop */
public class BreakerFlywheel extends BreakerGenericLoopedDevice implements BreakerGenericTestSuiteImplementation<BreakerFlywheelTestSuite> {
    //private BreakerPIDF flyPIDF;
    private double flywheelTargetRPM = 0;
    private WPI_TalonFX lFlyMotor;
    private WPI_TalonFX[] motors;
    //private BreakerFlywheelStateSpace flySS;
    private BreakerFlywheelTestSuite testSuite;
    private double lastVel = 0;
    private double accel;

    private double accelTol;
    private double velTol;
    
    private BreakerArbitraryFeedforwardProvider ffProvider;
    

    public BreakerFlywheel(BreakerFlywheelConfig config, WPI_TalonFX... flywheelMotors) {
        lFlyMotor = flywheelMotors[0];
        motors = flywheelMotors;

        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        talonConfig.slot0.kP = config.getkP();
        talonConfig.slot0.kI = config.getkI();
        talonConfig.slot0.kD = config.getkD();
        talonConfig.slot0.kF = config.getkF();
        talonConfig.slot0.closedLoopPeakOutput = 1.0;
        talonConfig.peakOutputForward = 1.0;
        talonConfig.peakOutputReverse = -1.0;
        talonConfig.voltageCompSaturation = 12.0;
        talonConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 1.5);
        BreakerCTREUtil.checkError(lFlyMotor.configAllSettings(talonConfig),
                " Failed to config swerve module drive motor ");
        lFlyMotor.selectProfileSlot(0, 0);
        lFlyMotor.set(ControlMode.Velocity, 0.0);
        
        for (int i = 1; i < motors.length; i++) {
            motors[i].follow(lFlyMotor, FollowerType.PercentOutput);
        }

        testSuite = new BreakerFlywheelTestSuite(this);

        velTol = config.getVelocityTolerence();
        accelTol = config.getAcclerationTolerence();

        ffProvider = config.getArbFFProvider();
    }

    public void setFlywheelSpeed(double flywheelTargetSpeedRPM) {
        flywheelTargetRPM = flywheelTargetSpeedRPM;
    }

    public double getFlywheelVelRSU() {
        return lFlyMotor.getSelectedSensorVelocity();
    }

    public double getFlywheelRPM() {
        return BreakerUnits.falconRSUtoRPM(getFlywheelVelRSU());
    }

    public double getFlywheelTargetRPM() {
        return flywheelTargetRPM;
    }

    /** sets flywheel speed to 0 RPM */
    public void stopFlywheel() {
        setFlywheelSpeed(0);
        BreakerLog.logSuperstructureEvent("flywheel stoped");
    }


    private void runFlywheel() {
        double flySetSpd = BreakerUnits.RPMtoFalconRSU(flywheelTargetRPM);
        double feedforward = ffProvider.getArbitraryFeedforwardValue(flywheelTargetRPM);
        lFlyMotor.set(ControlMode.Velocity, flySetSpd, DemandType.ArbitraryFeedForward, feedforward);
        accel = getFlywheelRPM() - lastVel;
        lastVel = getFlywheelRPM();
    }

    public boolean flywheelIsAtTargetVel() {
        return BreakerMath.isRoughlyEqualTo(flywheelTargetRPM, getFlywheelRPM(), velTol) && BreakerMath.isRoughlyEqualTo(accel, 0, accelTol);
    }

    @Override
    public void periodic() {
        runFlywheel();
    }

    @Override
    public BreakerFlywheelTestSuite getTestSuite() {
        return testSuite;
    }

    @Override
    public void runSelfTest() {
       faultStr = null;
       health = DeviceHealth.NOMINAL;
       for (WPI_TalonFX mot: motors) {
           Faults motFaults = new Faults();
           mot.getFaults(motFaults);
           Pair<DeviceHealth, String> faultData = BreakerCTREUtil.getMotorHealthAndFaults(motFaults);
           if (faultData.getFirst() != DeviceHealth.NOMINAL) {
               faultStr += faultData.getSecond();
               health = health != DeviceHealth.NOMINAL ? faultData.getFirst() : health;
           }
       }
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
    public boolean isUnderAutomaticControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public DevicePowerMode getPowerMode() {
        // TODO Auto-generated method stub
        return null;
    }
}
