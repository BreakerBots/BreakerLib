// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class BreakerTrajectoryUtil {
    public static List<Translation2d> toTranslationWaypointList(Translation2d... waypoints) {
        List<Translation2d> waypointList = new ArrayList<>();
        for (Translation2d trans: waypoints) {
            waypointList.add(trans);
        }
        return waypointList;
    }

    public static List<Pose2d> toPoseWaypointList(Pose2d... waypoints) {
        List<Pose2d> waypointList = new ArrayList<>();
        for (Pose2d pose: waypoints) {
            waypointList.add(pose);
        }
        return waypointList;
    }
}
