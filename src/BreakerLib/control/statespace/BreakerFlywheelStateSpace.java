// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.BreakerBots.lib.control.statespace;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;

public class BreakerFlywheelStateSpace extends SubsystemBase {
  /** Creates a new BreakerStateSpace. */
  private LinearSystem<N1, N1, N1> flywheelPlant;
  private KalmanFilter<N1, N1, N1> kalmanFilter;
  private LinearQuadraticRegulator<N1, N1, N1> lqrController;
  private LinearSystemLoop<N1, N1, N1> loop;
  private WPI_TalonFX leadFlywheelMotor;
  private double targetSpeedRadPerSec = 0;
  private double nextVoltage = 0;
  private boolean loopIsRunning = true;

  public BreakerFlywheelStateSpace(double flywheelGearing, double flywheelMomentOfInertiaJulesKgMetersSquared,
      double modelKalmanTrust,
      double encoderKalmanTrust, double lqrVelocityErrorTolerance, double lqrControlEffort,
      WPI_TalonFX... flywheelMotors) {
    flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(flywheelMotors.length),
        flywheelMomentOfInertiaJulesKgMetersSquared, flywheelGearing);
    kalmanFilter = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheelPlant, VecBuilder.fill(modelKalmanTrust),
        VecBuilder.fill(encoderKalmanTrust), 0.020);
    lqrController = new LinearQuadraticRegulator<>(flywheelPlant, VecBuilder.fill(lqrVelocityErrorTolerance),
        VecBuilder.fill(lqrControlEffort), 0.020);
    loop = new LinearSystemLoop<>(flywheelPlant, lqrController, kalmanFilter, 12.0, 0.020);
    leadFlywheelMotor = flywheelMotors[0];
  }

  /** Only call on robot start */
  public void reset() {
    loop.reset(VecBuilder.fill(BreakerUnits.falconVelRsuToRadPerSec(leadFlywheelMotor.getSelectedSensorVelocity())));
    targetSpeedRadPerSec = 0;
  }

  /** Sets target flywheel speed based on RPM. */
  public void setSpeedRPM(double targetSpeed) {
    targetSpeedRadPerSec = BreakerMath.radPerSecFromRPM(targetSpeed);
  }

  /** Sets target flywheel speed based on radians per second. */
  public void setSpeedRadPerSec(double targetSpeed) {
    targetSpeedRadPerSec = targetSpeed;
  }

  /** Sets the loop's target speed to zero */
  public void stop() {
    targetSpeedRadPerSec = 0;
  }

  /** Stops the state space loop from running */
  public void killLoop() {
    loopIsRunning = false;
  }

  /** restarts the state space loop if it has allredey been stoped */
  public void restartLoop() {
    loopIsRunning = true;
  }

  public boolean loopIsRunning() {
    return loopIsRunning;
  }

  private void runLoop() {
    if (loopIsRunning) {
      loop.setNextR(VecBuilder.fill(targetSpeedRadPerSec));
      loop.correct(
          VecBuilder.fill(BreakerUnits.falconVelRsuToRadPerSec(leadFlywheelMotor.getSelectedSensorVelocity())));
      loop.predict(0.020); // Predicts next cycle.
      nextVoltage = loop.getU(0);
    }
  }

  public double getNextVoltage() {
    return nextVoltage;
  }

  public double getNextPrecentSpeed() {
    return (nextVoltage / leadFlywheelMotor.getBusVoltage());
  }

  @Override
  public void periodic() {
    runLoop();
  }
}
