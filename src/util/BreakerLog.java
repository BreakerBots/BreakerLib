// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerSounds;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotMode;

/** Add your docs here. */
public class BreakerLog {

  public static final String breakerLibVersion = "V1.4";
  private static boolean usesOrchestra = false;
  private static BreakerFalconOrchestra orchestra;

  /** Commences logging. */
  public static void startLog(boolean autologNetworkTables) {
    DataLogManager.logNetworkTables(autologNetworkTables);
    DataLogManager.start();
    BreakerLog.usesOrchestra = false;
  }

  /** Commences logging and turns on Orchestra-controlled robot sounds. */
  public static void startLog(boolean autologNetworkTables, BreakerFalconOrchestra orchestra) {
    DataLogManager.logNetworkTables(autologNetworkTables);
    DataLogManager.start();
    BreakerLog.usesOrchestra = true;
    BreakerLog.orchestra = orchestra;
  }

  /** Startup message for robot. */
  public static void logRobotStarted(int teamNum, String teamName, String robotName, int year,
      String robotSoftwareVersion, String authorNames) {
    StringBuilder work = new StringBuilder(" | ---------------- ROBOT STARTED ---------------- |\n\n");
    work.append(" TEAM: " + teamNum + " - " + teamName + "\n");
    work.append(" ROBOT: " + robotName + " - " + year + "\n");
    work.append(" BREAKERLIB: " + breakerLibVersion + " | " + "ROBOT SOFTWARE: " + robotSoftwareVersion + "\n");
    work.append(" AUTHORS: " + authorNames + "\n\n");
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

  /** Needs justification */
  public static void logBreakerLibEvent(String event) {
    DataLogManager.log(" BREAKERLIB INTERNAL EVENT: " + event);
  }

  /** Logs errors... somehow. */
  public static void logError(String error) {
    DataLogManager.log(" ERROR: " + error);
  }
  public static void logError(Exception e) {
    DataLogManager.log(" ERROR: " + e.toString() + " : " + e.getStackTrace().toString());
  }

  /** Logs superstructure(?) events. */
  public static void logSuperstructureEvent(String event) {
    DataLogManager.log(" ROBOT SUPERSTRUCTURE EVENT: " + event);
  }

  /** Write custom message to log. */
  public static void log(String message) {
    DataLogManager.log(message);
  }
}
