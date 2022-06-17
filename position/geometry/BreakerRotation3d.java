// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/** Represents an object's angular orentation in 3-dimensional space using Euler Angles (yaw, pitch, and roll) */
public class BreakerRotation3d implements BreakerInterpolable<BreakerRotation3d> {

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

    public BreakerRotation3d(Rotation2d pitch, Rotation2d yaw) {
        this.pitch = pitch;
        this.yaw = yaw;
        roll = new Rotation2d();
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

    public BreakerRotation3d minus(BreakerRotation3d other) {
        Rotation2d newP = pitch.minus(other.getPitch());
        Rotation2d newY = yaw.minus(other.getYaw());
        Rotation2d newR = roll.minus(other.getRoll());
        return new BreakerRotation3d(newP, newY, newR);
    }

    @Override
    public String toString() {
        return "Yaw: " + yaw.toString() + ", Pitch: " + pitch.toString() + ", Roll: " + roll.toString();
    }

    @Override
    public BreakerRotation3d interpolate(double interpolendValue, double highKey, BreakerRotation3d highVal,
            double lowKey, BreakerRotation3d lowVal) {
        double interP = BreakerMath.interpolateLinear(interpolendValue, lowKey, highKey, lowVal.getPitch().getRadians(), highVal.getPitch().getRadians());
        double interY = BreakerMath.interpolateLinear(interpolendValue, lowKey, highKey, lowVal.getYaw().getRadians(), highVal.getYaw().getRadians());
        double interR = BreakerMath.interpolateLinear(interpolendValue, lowKey, highKey, lowVal.getRoll().getRadians(), highVal.getRoll().getRadians());
        return new BreakerRotation3d(new Rotation2d(interP), new Rotation2d(interY), new Rotation2d(interR));
    }

    @Override
    public BreakerRotation3d getSelf() {
        return this;
    }

    /** [0] = pitchRad, [1] = yawRad, [2] = rollRad */
    @Override
    public double[] getInterpolatableData() {
        return new double[] {pitch.getRadians(), yaw.getRadians(), roll.getRadians()};
    }

    @Override
    public BreakerRotation3d fromInterpolatableData(double[] interpolatableData) {
        return new BreakerRotation3d(new Rotation2d(interpolatableData[0]), new Rotation2d(interpolatableData[1]), new Rotation2d(interpolatableData[2]));
    }
}
