// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

import javax.print.attribute.standard.MediaSize.Other;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Tracer;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/**
 * represents a point with 2 axial vectors of ajustable magnatudes (one paralell
 * with the point's X, and Y axis)
 */
public class BreakerVector2 implements BreakerInterpolable<BreakerVector2> {
    private Rotation2d forceRotation;
    private double force;
    private double forceX;
    private double forceY;

    /**
     * creates a new BreakerVector2 from the magnatudes of the vectors X and Y
     * components
     */
    public BreakerVector2(double forceX, double forceY) {
        this.forceX = forceX;
        this.forceY = forceY;
        forceRotation = new Rotation2d(Math.atan2(forceY, forceX));
        force = Math.sqrt(Math.pow(forceX, 2) + Math.pow(forceY, 2));
    }

    private BreakerVector2(double forceX, double forceY, double force, Rotation2d forceRotation) {
        this.forceX = forceX;
        this.forceY = forceY;
        this.force = force;
        this.forceRotation = forceRotation;
    }

    /** creates an empty BreakerVector2 with 0's for all values */
    public BreakerVector2() {
        forceX = 0;
        forceY = 0;
        forceRotation = Rotation2d.fromDegrees(0);
        force = 0;
    }

    /**
     * creates a new BreakerVector2 from the vectors Magnatude and the vectors angle
     * in the Yaw angular axis
     */
    public static BreakerVector2 fromForceAndRotation(Rotation2d forceRotation, double force) {
        return new BreakerVector2(force * Math.cos(forceRotation.getRadians()),
                force * Math.sin(forceRotation.getRadians()), force, forceRotation);
    }

    public double getForce() {
        return force;
    }

    public Rotation2d getForceRotation() {
        return forceRotation;
    }

    public double getForceX() {
        return forceX;
    }

    public double getForceY() {
        return forceY;
    }

    public BreakerVector2 rotateBy(Rotation2d rotation) {
        double cos = Math.cos(rotation.getRadians());
        double sin = Math.sin(rotation.getRadians());
        return new BreakerVector2((this.forceX * cos) - (this.forceY * sin), (this.forceX * sin) + (this.forceY * cos));
    }

    @Override
    public boolean equals(Object obj) {
        return (Math.abs(((BreakerVector2) obj).forceX - forceX) < 1E-9)
                && (Math.abs(((BreakerVector2) obj).forceY - forceY) < 1E-9);
    }

    @Override
    public BreakerVector2 interpolate(double interpolendValue, double highKey, BreakerVector2 highVal,
            double lowKey, BreakerVector2 lowVal) {
        double interX = BreakerMath.interpolateLinear(interpolendValue, highKey, lowKey, lowVal.getForceX(),
                highVal.getForceX());
        double interY = BreakerMath.interpolateLinear(interpolendValue, highKey, lowKey, lowVal.getForceY(),
                highVal.getForceY());
        return new BreakerVector2(interX, interY);
    }

    @Override
    public BreakerVector2 getSelf() {
        return this;
    }

    /** [0] = X, [1] = Y */
    @Override
    public double[] getInterpolatableData() {
        return new double[] { forceX, forceY };
    }

    @Override
    public BreakerVector2 fromInterpolatableData(double[] interpolatableData) {
        return new BreakerVector2(interpolatableData[0], interpolatableData[1]);
    }

    @Override
    public String toString() {
       return "Vector Magnatude:" + force + ", X-Magnatude: " + forceX + ", Y-Magnatude: " + forceY + ", Vector Angle: " + forceRotation.toString();
    }
}
