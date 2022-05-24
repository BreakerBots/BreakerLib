// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.geometry;

import edu.wpi.first.math.geometry.Translation2d;

/** Represents an objects position in the X, Y, and Z axies */
public class BreakerTranslation3d {

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
}
