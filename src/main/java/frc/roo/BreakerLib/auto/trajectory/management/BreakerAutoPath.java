// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public class BreakerAutoPath {
    private SequentialCommandGroup autoPath;
    private String pathName;
    public BreakerAutoPath(String pathName, SequentialCommandGroup autoPath) {
        this.autoPath = autoPath;
        this.pathName = pathName;
    }

    public String getPathName() {
        return pathName;
    }

    public SequentialCommandGroup getBaseCommandGroup() {
        return autoPath;
    }

    public SequentialCommandGroup startPath() {
        autoPath.schedule();
        return autoPath;
    }
}
