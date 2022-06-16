
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import static edu.wpi.first.wpilibj.RobotState.*;

/**
 * RoboRIO wrapper class utilizing Subsystem framework. Manages time and robot
 * mode.
 */
public class BreakerRoboRIO extends SubsystemBase {

    /** Operating modes for robot. */
    public enum RobotMode {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST,
        UNKNOWN
    }

    // RoboRIO object vars.
    private double prevTime = RobotController.getFPGATime();
    private double diffTime = 0;
    private RobotMode currentMode = RobotMode.UNKNOWN;
    private RobotMode prevMode = RobotMode.UNKNOWN;
    private boolean prevBrownoutState = false;
    private boolean curBrownoutState = false;
    private int brownoutNum = 0;

    // Static RoboRIO object.
    private static BreakerRoboRIO roboRIO = new BreakerRoboRIO();

    // RoboRIO object methods.

    /** Calculates time between cycles. Runs periodically. */
    private double calculateInterCycleTime() {
        double curTime = RobotController.getFPGATime(); // In microseconds
        double diffTime = curTime - prevTime;
        prevTime = curTime;
        return BreakerUnits.microsecondsToSeconds(diffTime);
    }

    /** Updates robot mode using {@link RobotState}. */
    private void updateRobotMode() {
        roboRIO.prevMode = roboRIO.currentMode;
        if (isDisabled()) {
            roboRIO.currentMode = RobotMode.DISABLED;
        } else if (isTeleop()) {
            roboRIO.currentMode = RobotMode.TELEOP;
        } else if (isAutonomous()) {
            roboRIO.currentMode = RobotMode.AUTONOMOUS;
        } else {
            roboRIO.currentMode = RobotMode.TEST;
        }
        if (robotModeHasChanged()) {
            BreakerLog.logRobotChangedMode(roboRIO.currentMode);
        }
    }

    @Override
    public void periodic() {
        diffTime = calculateInterCycleTime();
        updateRobotMode();
        checkBrownout();
    }

    // Static getters.

    /** Returns time between cycles in seconds. */
    public static double getInterCycleTimeSeconds() {
        return roboRIO.diffTime;
    }

    /** Returns time as long of microseconds */
    public static long getRobotTimeMCRS() {
        return RobotController.getFPGATime();
    }

    /** Operating time of robot in seconds. */
    public static double getRobotTimeSeconds() {
        return BreakerUnits.microsecondsToSeconds(RobotController.getFPGATime());
    }

    /** Returns current operating mode of robot. */
    public static RobotMode getCurrentRobotMode() {
        return roboRIO.currentMode;
    }

    /** Checks if the RoboRIO's state has changed since the last cycle. */
    public static boolean robotModeHasChanged() {
        return roboRIO.currentMode != roboRIO.prevMode;
    }

    private void checkBrownout() {
        prevBrownoutState = curBrownoutState;
        curBrownoutState = RobotController.isBrownedOut();
        if (curBrownoutState != prevBrownoutState) {
            if (curBrownoutState) {
                brownoutNum ++;
                BreakerLog.logEvent(" ROBORIO BROWNOUT DETECTED! ( #" + brownoutNum + ")");
            } else {
                BreakerLog.logEvent(" ROBORIO BROWNOUT ENDED! ( #" + brownoutNum + ")");
            }
        }
    }
}
