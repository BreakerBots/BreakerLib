// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management;

import edu.wpi.first.wpilibj2.command.Command;

/** Represnets a Auto Path, contans the path's base {@link Command} and its name */
public class BreakerAutoPath {

    private Command autoPath;
    private String pathName;

    /** Creates a BreakerAutoPath.
     * 
     * @param pathName Name of the path.
     * @param autoPath Autopath command.
     */
    public BreakerAutoPath(String pathName, Command autoPath) {
        this.autoPath = autoPath;
        this.pathName = pathName;
    }

    public String getPathName() {
        return pathName;
    }

    public Command getBaseAutoPath() {
        return autoPath;
    }

    /** Schedules base auto path. */
    public Command startPath() {
        autoPath.schedule();
        return autoPath;
    }
}
