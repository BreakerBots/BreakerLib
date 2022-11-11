// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.logging;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerSounds;
import frc.robot.BreakerLib.util.BreakerLibVersion;
import frc.robot.BreakerLib.util.BreakerRoboRIO;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotMode;
import frc.robot.BreakerLib.util.robotmanager.BreakerRobotStartConfig;

/** Global log manager for all your data logging and printing needs */
public class BreakerLog {

  private static boolean usesOrchestra = false;
  private static BreakerFalconOrchestra orchestra;

  /**
   * Commences logging.
   * 
   * @param autologNetworkTables True to log NetworkTables data, false to not log
   *                             data.
   */
  public static void startLog(boolean autologNetworkTables) {
    DataLogManager.logNetworkTables(autologNetworkTables);
    DataLogManager.start();
    BreakerLog.usesOrchestra = false;
  }

  /**
   * Commences logging and turns on Orchestra-controlled robot sounds.
   * 
   * @param autologNetworkTables True to log NetworkTables data, false to not log
   *                             data.
   * @param orchestra            BreakerFalconOrchestra to use.
   */
  public static void startLog(boolean autologNetworkTables, BreakerFalconOrchestra orchestra) {
    DataLogManager.logNetworkTables(autologNetworkTables);
    DataLogManager.start();
    BreakerLog.usesOrchestra = true;
    BreakerLog.orchestra = orchestra;
  }

  /** Startup message for robot. */
  public static void logRobotStarted(BreakerRobotStartConfig startConfig) {
    StringBuilder work = new StringBuilder(" | ---------------- ROBOT STARTED ---------------- |\n\n");
    work.append(" TEAM: " + startConfig.getTeamNum() + " - " + startConfig.getTeamName() + "\n");
    work.append(" ROBOT: " + startConfig.getRobotName() + " - " + startConfig.getRobotYear() + "\n");
    work.append(" BREAKERLIB: " + BreakerLibVersion.Version + " | " + "ROBOT SOFTWARE: "
        + startConfig.getRobotSoftwareVersion() + "\n");
    work.append(" AUTHORS: " + startConfig.getAuthorNames() + "\n\n");
    work.append(" | ---------------------------------------------- | \n\n\n");
    BreakerLog.log(work.toString());
  }

  /**
   * Logs robot mode change and plays enable tune. (automatically called by
   * BreakerRoboRIO)
   */
  public static void logRobotChangedMode(RobotMode newMode) {
    DataLogManager.log("| ---- ROBOT MODE CHANGED TO: " + newMode + " ---- |");
    if (usesOrchestra && BreakerRoboRIO.robotModeHasChanged() && newMode != RobotMode.DISABLED) {
      orchestra.loadMusic(BreakerSounds.enableSound);
      orchestra.playMusic();
    }
  }

  /** Logs given event. */
  public static void logEvent(String event) {
    DataLogManager.log(" EVENT: " + event);
  }

  /**
   * Internal logging function for breakerlib classes to sepreate automated
   * breakerlib logging from user loging
   */
  public static void logBreakerLibEvent(String event) {
    DataLogManager.log(" BREAKERLIB INTERNAL EVENT: " + event);
  }

  /**
   * Logs either exceptions thrown by code, or suer difigned errors either from
   * code or physical errors
   */
  public static void logError(String error) {
    DataLogManager.log(" ERROR: " + error);
  }

  public static void logError(Exception e) {
    DataLogManager.log(" ERROR: " + e.toString() + " : " + e.getStackTrace().toString());
  }

  /**
   * Logs robot superstructure (physical) events (i.e. intake activated, shooter
   * enabled)
   */
  public static void logSuperstructureEvent(String event) {
    DataLogManager.log(" ROBOT SUPERSTRUCTURE EVENT: " + event);
  }

  /** Write custom message to log. */
  public static void log(String message) {
    DataLogManager.log(message);
  }
}
