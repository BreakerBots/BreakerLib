// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystemcores.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.control.statespace.BreakerFlywheelStateSpace;
import frc.robot.BreakerLib.util.BreakerLog;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/** A class representing a robot's shooter flywheel and its assocated controle loop */
public class BreakerFlywheel extends SubsystemBase {
    private PIDController flyPID;
    private boolean runFlywheel = false;
    private double flywheelTargetRSU = 0;
    private MotorControllerGroup flywheel;
    private WPI_TalonFX lFlyMotor;
    private BreakerFlywheelStateSpace flySS;

    public BreakerFlywheel(BreakerFlywheelConfig config, WPI_TalonFX... flywheelMotors) {
        flyPID = new PIDController(config.getFlywheelKp(), config.getFlywheelKi(), config.getFlywheelKd());
        flyPID.setTolerance(config.getFlywheelPosTol(), config.getFlywheelVelTol());
        flySS = new BreakerFlywheelStateSpace(config.getFlywheelGearing(),
                config.getFlywheelMomentOfInertiaJulesKgMetersSquared(), config.getModelKalmanTrust(),
                config.getEncoderKalmanTrust(), config.getLqrVelocityErrorTolerance(), config.getLqrControlEffort(),
                flywheelMotors);
        flywheel = new MotorControllerGroup(flywheelMotors);
        lFlyMotor = flywheelMotors[0];
    }

    public void setFlywheelSpeed(double flywheelTargetSpeedRPM) {
        flywheelTargetRSU = BreakerUnits.RPMtoFalconRSU(flywheelTargetSpeedRPM);
    }

    public double getFlywheelVelRSU() {
        return lFlyMotor.getSelectedSensorVelocity();
    }

    public double getFlywheelTargetVelRSU() {
        return flywheelTargetRSU;
    }

    public void stopFlywheel() {
        runFlywheel = false;
        flySS.killLoop();
        flywheel.set(0);
        BreakerLog.logSuperstructureEvent("flywheel stoped");
    }

    public void startFlywheel() {
        runFlywheel = true;
        flySS.restartLoop();
        BreakerLog.logSuperstructureEvent("flywheel started charging");
    }

    private void runFlywheel() {
        if (runFlywheel) {
            flySS.setSpeedRPM(BreakerUnits.falconRSUtoRPM(flywheelTargetRSU));
            double flySetSpd = flyPID.calculate(getFlywheelVelRSU(), flywheelTargetRSU) + flySS.getNextPrecentSpeed();
            flywheel.set(flySetSpd);
        }
    }

    public boolean flywheelIsAtTargetVel() {
        return flyPID.atSetpoint();
    }

    @Override
    public void periodic() {
        runFlywheel();
    }
}
