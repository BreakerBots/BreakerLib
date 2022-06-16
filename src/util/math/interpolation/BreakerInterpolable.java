// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

/** Add your docs here. */
public interface BreakerInterpolable<V> {

    /**
     * Linearly interpolates between 2 points based on a known X value
     * 
     * @param interpolendValue X-value to interpolate to.
     * @param highKey          X-value of high point.
     * @param highVal          Output(Y) value of high point.
     * @param lowKey           X-value of low point.
     * @param lowVal           Output(Y) value of low point.
     * 
     * @return Implemented object interpolated based on the given points.
     */
    public abstract V interpolate(double interpolendValue, double highKey, V highVal, double lowKey, V lowVal);

    public abstract V getSelf();

    public abstract double[] getInterpolatableData();

    public abstract V fromInterpolatableData(double[] interpolatableData);
}
