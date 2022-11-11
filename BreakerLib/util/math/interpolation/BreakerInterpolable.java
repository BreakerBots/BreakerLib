// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

import edu.wpi.first.math.interpolation.Interpolatable;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** wrapper around WPILib's {@link Interpolatable} interface that supports non-linear interpolation */
public interface BreakerInterpolable<V> extends Interpolatable<V> {

    public abstract double[] getInterpolatableData();

    public abstract V fromInterpolatableData(double[] interpolatableData);

    public default V interpolate(double query, double lowKey, double highKey, V highVal) {
        return interpolate(highVal, BreakerMath.getLerpT(query, lowKey, highKey));
    }
}
