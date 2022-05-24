// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

import java.util.TreeMap;
import java.util.Map.Entry;

/** Add your docs here. */
public class BreakerInterpolateingDoubleTreeMap extends BreakerInterpolateable {
    private TreeMap<Double, Double> indexesAndVectors;
    public BreakerInterpolateingDoubleTreeMap(TreeMap<Double, Double> indexesAndVectors) {
        this.indexesAndVectors = indexesAndVectors;
    }

    public double getInterpolatedDouble(Double interpolendValue) {
        Entry<Double, Double> low = indexesAndVectors.floorEntry(interpolendValue);
        Entry<Double, Double> high = indexesAndVectors.ceilingEntry(interpolendValue);

        if (low == null) {
            return high.getValue();
        }
        if (high == null) {
            return low.getValue();
        }
        if (high.getValue().equals(low.getValue())) {
            return high.getValue();
        }

        return interpolateLinear(interpolendValue, low.getKey(), high.getKey(), low.getValue(), high.getValue());
    }
}
