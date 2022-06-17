// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

import frc.robot.BreakerLib.util.math.BreakerMath;

/**
 * Wraps around the double primitive type for use with BreakerLib's
 * interpolation classes
 */
public class BreakerInterpolableDouble implements BreakerInterpolable<BreakerInterpolableDouble> {
    private double value;

    public BreakerInterpolableDouble(Double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }

    @Override
    public BreakerInterpolableDouble getSelf() {
        return this;
    }

    @Override
    public BreakerInterpolableDouble interpolate(double valToInterpolate, double highKey,
            BreakerInterpolableDouble highVal, double lowKey, BreakerInterpolableDouble lowVal) {
        BreakerMath.interpolateLinear(valToInterpolate, lowKey, highKey, lowVal.getValue(), highVal.getValue());
        return new BreakerInterpolableDouble(value);
    }

    @Override
    public double[] getInterpolatableData() {
        return new double[] { value };
    }

    @Override
    public BreakerInterpolableDouble fromInterpolatableData(double[] interpolatableData) {
        return new BreakerInterpolableDouble(interpolatableData[0]);
    }

}
