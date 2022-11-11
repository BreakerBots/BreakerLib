// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robotmanager;

/** Configures general robot info on startup. */
public class BreakerRobotStartConfig {
    private int teamNum;
    private String teamName;
    private String robotName;
    private int robotYear;
    private String robotSoftwareVersion;
    private String authorNames;

    /**
     * Constructs a BreakerStartConfig. Info will appear in startup message.
     * 
     * @param teamNum FRC team number.
     * @param teamName FRC team name.
     * @param robotName Name of your robot.
     * @param robotYear Competition year your robot was made for.
     * @param robotSoftwareVersion Robot software version.
     * @param authorNames Names of code authors. 
     */
    public BreakerRobotStartConfig(int teamNum, String teamName, String robotName, int robotYear, String robotSoftwareVersion, String authorNames) {
        this.teamNum = teamNum;
        this.teamName = teamName;
        this.robotName = robotName;
        this.robotYear = robotYear;
        this.robotSoftwareVersion = robotSoftwareVersion;
        this.authorNames = authorNames;
    }

    /** Creates a default start configuration. Not for general use. */
    public BreakerRobotStartConfig() {
        this.teamNum = 0000;
        this.teamName = "FRC Team";
        this.robotName = "FRC Robot";
        this.robotYear = 9999;
        this.robotSoftwareVersion = "V0.0";
        this.authorNames = "Unknown";
    }

    /** @return Code author names. */
    public String getAuthorNames() {
        return authorNames;
    }

    /** @return Name of robot. */
    public String getRobotName() {
        return robotName;
    }

    /** @return Robot software version. */
    public String getRobotSoftwareVersion() {
        return robotSoftwareVersion;
    }

    /** @return Robot's game year. */
    public int getRobotYear() {
        return robotYear;
    }

    /** @return Team name. */
    public String getTeamName() {
        return teamName;
    }

    /** @return Team number. */
    public int getTeamNum() {
        return teamNum;
    }

}
