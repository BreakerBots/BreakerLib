// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robotmanager;

/** Add your docs here. */
public class BreakerRobotStartConfig {
    private int teamNum;
    private String teamName;
    private String robotName;
    private int robotYear;
    private String robotSoftwareVersion;
    private String authorNames;

    public BreakerRobotStartConfig(int teamNum, String teamName, String robotName, int robotYear, String robotSoftwareVersion, String authorNames) {
        this.teamNum = teamNum;
        this.teamName = teamName;
        this.robotName = robotName;
        this.robotYear = robotYear;
        this.robotSoftwareVersion = robotSoftwareVersion;
        this.authorNames = authorNames;
    }

    public BreakerRobotStartConfig() {
        this.teamNum = 0000;
        this.teamName = "FRC Team";
        this.robotName = "FRC Robot";
        this.robotYear = 9999;
        this.robotSoftwareVersion = "V0.0";
        this.authorNames = "People";
    }

    public String getAuthorNames() {
        return authorNames;
    }

    public String getRobotName() {
        return robotName;
    }

    public String getRobotSoftwareVersion() {
        return robotSoftwareVersion;
    }

    public int getRobotYear() {
        return robotYear;
    }

    public String getTeamName() {
        return teamName;
    }

    public int getTeamNum() {
        return teamNum;
    }

}
