// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

import java.util.Map.Entry;
import java.util.TreeMap;

import frc.robot.BreakerLib.physics.BreakerVector3;

/** Add your docs here. */
public class BreakerInterpolateingVector3TreeMap extends BreakerInterpolateable {
    private TreeMap<Double, BreakerVector3> indexesAndVectors;
    public BreakerInterpolateingVector3TreeMap(TreeMap<Double, BreakerVector3> indexesAndVectors) {
        this.indexesAndVectors = indexesAndVectors;
    }

    public BreakerVector3 getInterpolatedVector3(Double interpolendValue) {
        Entry<Double, BreakerVector3> low = indexesAndVectors.floorEntry(interpolendValue);
        Entry<Double, BreakerVector3> high = indexesAndVectors.ceilingEntry(interpolendValue);

        if (low == null) {
            return high.getValue();
        }
        if (high == null) {
            return low.getValue();
        }
        if (high.getValue().equals(low.getValue())) {
            return high.getValue();
        }

        double interX = interpolateLinear(interpolendValue, low.getKey(), high.getKey(), low.getValue().getForceX(), high.getValue().getForceX());
        double interY = interpolateLinear(interpolendValue, low.getKey(), high.getKey(), low.getValue().getForceY(), high.getValue().getForceZ());
        double interZ = interpolateLinear(interpolendValue, low.getKey(), high.getKey(), low.getValue().getForceZ(), high.getValue().getForceZ());

        return new BreakerVector3(interX, interY, interZ);
    }
}
