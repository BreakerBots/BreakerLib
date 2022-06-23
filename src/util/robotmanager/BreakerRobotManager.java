// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robotmanager;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoManager;
import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoPath;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;
import frc.robot.BreakerLib.factories.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.BreakerLog;
import frc.robot.BreakerLib.util.selftest.SelfTest;

/** Add your docs here. */
public class BreakerRobotManager {
    private static SelfTest test;
    private static BreakerAutoManager autoManager;

    public static void setup(BreakerRobotConfig robotConfig) {
        if (robotConfig.UsesOrchestra()) { 
            BreakerLog.startLog(robotConfig.getAutologNetworkTables(), robotConfig.getOrchestra());
            test = new SelfTest(robotConfig.getSecondsBetweenSelfChecks(), robotConfig.getSelftestServerAddress(), robotConfig.getOrchestra(), robotConfig.getAutoRegesterDevices());
        } else {
            BreakerLog.startLog(robotConfig.getAutologNetworkTables());
            test = new SelfTest(robotConfig.getSecondsBetweenSelfChecks(), robotConfig.getSelftestServerAddress(), robotConfig.getAutoRegesterDevices());
        }
        autoManager = robotConfig.UsesPaths() ? new BreakerAutoManager(robotConfig.getAutoPaths()) : new BreakerAutoManager();
        BreakerLog.logRobotStarted(robotConfig.getStartConfig());
    }

    public static SelfTest getSelfTest() {
        return test;
    }

    public static BreakerAutoManager getAutoManager() {
        return autoManager;
    }

    public static SequentialCommandGroup getSelectedAutoPath() {
        return autoManager.getSelectedBaseCommandGroup();
    }
}
