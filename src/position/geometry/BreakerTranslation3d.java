// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/** Represents an objects position in the X, Y, and Z axies */
public class BreakerTranslation3d implements BreakerInterpolable<BreakerTranslation3d> {

    // Position values in meters
    private double metersX;
    private double metersY;
    private double metersZ;

    /**
     * Creates a new BreakerTranslation3d with given distance parameters.
     * <p>
     * Parameters follow field coordinate system.
     * 
     * @param metersX Meters in x-position.
     * @param metersY Meters in y-position.
     * @param metersZ Meters in z-position.
     */
    public BreakerTranslation3d(double metersX, double metersY, double metersZ) {
        this.metersX = metersX;
        this.metersY = metersY;
        this.metersZ = metersZ;
    }

    /** Returns 2d translation of x and y position. */
    public Translation2d get2dTranslationComponent() {
        return new Translation2d(metersX, metersY);
    }

    public double getMetersX() {
        return metersX;
    }

    public double getMetersY() {
        return metersY;
    }

    public double getMetersZ() {
        return metersZ;
    }

    /** Returns new BreakerTranslation3d transformed by another BreakerTranslation3d. 
     * 
     * @other Other BreakerTranslation3d, position values are added to current BreakerTranslation3d.
    */
    public BreakerTranslation3d plus(BreakerTranslation3d other) {
        return new BreakerTranslation3d(this.metersX + other.metersX, this.metersY + other.metersY,
                this.metersZ + other.metersZ);
    }

    @Override
    public String toString() {
        return "X: " + metersX + ", Y: " + metersY + ", Z: " + metersZ;
    }

    @Override
    public BreakerTranslation3d interpolate(double valToInterpolate, double highKey, BreakerTranslation3d highVal,
            double lowKey, BreakerTranslation3d lowVal) {
                double interX = BreakerMath.interpolateLinear(valToInterpolate, lowKey, highKey, lowVal.getMetersX(), highVal.getMetersX());
                double interY = BreakerMath.interpolateLinear(valToInterpolate, lowKey, highKey, lowVal.getMetersY(), highVal.getMetersZ());
                double interZ = BreakerMath.interpolateLinear(valToInterpolate, lowKey, highKey, lowVal.getMetersZ(), highVal.getMetersZ());
        
                return new BreakerTranslation3d(interX, interY, interZ);
    }

    @Override
    public BreakerTranslation3d getSelf() {
        return this;
    }

    /** [0] = X, [1] = Y, [2] = Z */
    @Override
    public double[] getInterpolatableData() {
        return new double[] {metersX, metersY, metersZ};
    }

    @Override
    public BreakerTranslation3d fromInterpolatableData(double[] interpolatableData) {
        return new BreakerTranslation3d(interpolatableData[0], interpolatableData[1], interpolatableData[2]);
    }
}
