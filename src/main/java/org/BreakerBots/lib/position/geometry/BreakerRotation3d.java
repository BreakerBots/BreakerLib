// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Represents an object's angular orentation in 3-dimensional space using yaw, pitch, and roll */
public class BreakerRotation3d {

    private Rotation2d pitch;
    private Rotation2d yaw;
    private Rotation2d roll;

    /** Creates a new BreakerRotation3d.
     * 
     * @param pitch Pitch angle as Rotation2d.
     * @param yaw Yaw angle as Rotation2d.
     * @param roll Roll angle as Rotation2d.
     */
    public BreakerRotation3d(Rotation2d pitch, Rotation2d yaw, Rotation2d roll) {
        this.pitch = pitch;
        this.yaw = yaw;
        this.roll = roll;
    }

    public Rotation2d getPitch() {
        return pitch;
    }

    public Rotation2d getYaw() {
        return yaw;
    }

    public Rotation2d getRoll() {
        return roll;
    }

    /** Returns new Rotation3d with values based on the sum of this and other Rotation3d.  */
    public BreakerRotation3d plus(BreakerRotation3d other) {
        Rotation2d newP = pitch.plus(other.getPitch());
        Rotation2d newY = yaw.plus(other.getYaw());
        Rotation2d newR = roll.plus(other.getRoll());
        return new BreakerRotation3d(newP, newY, newR);
    }
}
