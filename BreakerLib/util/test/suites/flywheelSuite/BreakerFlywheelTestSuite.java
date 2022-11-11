// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.suites.flywheelSuite;

import frc.robot.BreakerLib.subsystem.cores.shooter.BreakerFlywheel;
import frc.robot.BreakerLib.util.test.suites.BreakerTestBase;
import frc.robot.BreakerLib.util.test.suites.BreakerTestSuiteDataLogType;

/** Add your docs here. */
public class BreakerFlywheelTestSuite {
    private BreakerFlywheel baseFlywheel;
    private BreakerTestSuiteDataLogType logType;
    public BreakerFlywheelTestSuite(BreakerFlywheel baseFlywheel) {
        this.baseFlywheel = baseFlywheel;
    }

    public void setLogType(BreakerTestSuiteDataLogType newLogType) {
        logType = newLogType;
    }

    public BreakerTimedFlywheelSpinUpTest timedSpinUpTest(double testTimeSeconds, double targetRPM) {
        BreakerTimedFlywheelSpinUpTest test = new BreakerTimedFlywheelSpinUpTest(testTimeSeconds, targetRPM, baseFlywheel, logType);
        test.schedule();
        return test;
    }

    public BreakerRepeatedFlywheelChargeCycleTest repeatedChargeCycleTest(int numberOfCycles, double targetRPM) {
        BreakerRepeatedFlywheelChargeCycleTest test = new BreakerRepeatedFlywheelChargeCycleTest(numberOfCycles, targetRPM, baseFlywheel, logType);
        test.schedule();
        return test;
    }

}
