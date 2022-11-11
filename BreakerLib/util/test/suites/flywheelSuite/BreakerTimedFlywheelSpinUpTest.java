// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.suites.flywheelSuite;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.BreakerLib.subsystem.cores.shooter.BreakerFlywheel;
import frc.robot.BreakerLib.util.math.averages.BreakerAverage;
import frc.robot.BreakerLib.util.math.averages.BreakerRunningAverage;
import frc.robot.BreakerLib.util.test.suites.BreakerTestBase;
import frc.robot.BreakerLib.util.test.suites.BreakerTestSuiteDataLogType;

public class BreakerTimedFlywheelSpinUpTest extends BreakerTestBase {
  private double testTimeSeconds, targetRPM, startTime, timeToStabilitySec, peakDiv, totalCycles, stableCycles;
  private BreakerFlywheel baseFlywheel;
  private BreakerAverage avgSpeed = new BreakerAverage();
  private List<Pair<Double, Double>> rpmAndTimestamps = new ArrayList<>();
  public BreakerTimedFlywheelSpinUpTest(double testTimeSeconds, double targetRPM, BreakerFlywheel baseFlywheel, BreakerTestSuiteDataLogType logType) {
    super(logType, "Timed_Flywheel_Spin_Up", "Test_Time_Seconds: " + testTimeSeconds);
    this.targetRPM = targetRPM; 
    this.testTimeSeconds = testTimeSeconds;
    this.baseFlywheel = baseFlywheel;
    startTime = 0;
    timeToStabilitySec = 0;
    peakDiv = 0;
    totalCycles = 0;
    stableCycles = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logStart();
    baseFlywheel.setFlywheelSpeed(targetRPM);
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    totalCycles ++;
    double curVel = baseFlywheel.getFlywheelRPM();
    double curTime = Timer.getFPGATimestamp();
    if (baseFlywheel.flywheelIsAtTargetVel()) {
      if(stableCycles == 0) {
        timeToStabilitySec = curTime - startTime;
      }
      stableCycles ++;
    }
    if ((targetRPM - curVel) > peakDiv) {
      peakDiv = targetRPM - curVel;
    }
    avgSpeed.addValue(baseFlywheel.getFlywheelRPM());
    rpmAndTimestamps.add(new Pair<Double,Double>(curVel, curTime));
    periodicLog("Cur Speed: " + curVel + ", At Target: " + baseFlywheel.flywheelIsAtTargetVel() + ", Stable Time Sec: " + (stableCycles * 0.02));
  }

  public BreakerTimedFlywheelSpinUpTestResult getResults() {
    return new BreakerTimedFlywheelSpinUpTestResult(targetRPM, testTimeSeconds, timeToStabilitySec, peakDiv, stableCycles / totalCycles, avgSpeed, rpmAndTimestamps);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logEnd(getResults().toString());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= (startTime + testTimeSeconds);
  }

  public class BreakerTimedFlywheelSpinUpTestResult {
    private double targetRPM, testTimeSeconds, timeToStablitySeconds, peakDeveationRPM, precentOfTimeStable;
    private BreakerAverage averageSpeeds;
    private List<Pair<Double, Double>> rpmsAndTimestamps;
    protected BreakerTimedFlywheelSpinUpTestResult(double targetRPM, double testTimeSeconds, double timeToStablitySeconds, double peakDeveationRPM, double precentOfTimeStable, BreakerAverage averageSpeeds, List<Pair<Double, Double>> rpmsAndTimestamps) {
      this.targetRPM = targetRPM;
      this.testTimeSeconds = testTimeSeconds;
      this.timeToStablitySeconds = timeToStablitySeconds;
      this.peakDeveationRPM = peakDeveationRPM;
      this.precentOfTimeStable = precentOfTimeStable;
      this.averageSpeeds = averageSpeeds;
      this.rpmsAndTimestamps = rpmAndTimestamps;
    }

    public BreakerAverage getAverageSpeeds() {
        return averageSpeeds;
    }

    public double getPeakDeveationRPM() {
        return peakDeveationRPM;
    }

    public double getPrecentOfTimeStable() {
        return precentOfTimeStable;
    }

    public List<Pair<Double, Double>> getRpmsAndTimestamps() {
        return rpmsAndTimestamps;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getTestTimeSeconds() {
        return testTimeSeconds;
    }

    public double getTimeToStablitySeconds() {
        return timeToStablitySeconds;
    }

    public double getAverageDeveation() {
      return targetRPM - averageSpeeds.getAverage();
    }

    @Override
    public String toString() {
        return "Target RPM: " + targetRPM + " | Average RPM: " + averageSpeeds.getAverage() +
          "\n | Avg Deveation RPM: " + getAverageDeveation() + " | Peak Deveation RPM: " + peakDeveationRPM +
          "\n | Total Test Seconds: " + testTimeSeconds + " | Time To Stablity: " + timeToStabilitySec +
          "\n | Prec Of Time Stable: " + precentOfTimeStable;
    }
  }
}
