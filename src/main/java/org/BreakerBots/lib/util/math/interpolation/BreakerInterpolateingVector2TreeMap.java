// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

import java.util.HashMap;
import java.util.TreeMap;
import java.util.Map.Entry;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.BreakerLib.physics.BreakerVector2;

/** Add your docs here. */
public class BreakerInterpolateingVector2TreeMap extends BreakerInterpolateable {
    private TreeMap<Double, BreakerVector2> indexesAndVectors;
    public BreakerInterpolateingVector2TreeMap(TreeMap<Double, BreakerVector2> indexesAndVectors) {
        this.indexesAndVectors = indexesAndVectors;
    }

    public BreakerVector2 getInterpolatedVector2(Double interpolendValue) {
        Entry<Double, BreakerVector2> low = indexesAndVectors.floorEntry(interpolendValue);
        Entry<Double, BreakerVector2> high = indexesAndVectors.ceilingEntry(interpolendValue);

        if (low == null) {
            return high.getValue();
        }
        if (high == null) {
            return low.getValue();
        }
        if (high.getValue().equals(low.getValue())) {
            return high.getValue();
        }

        double interF = interpolateLinear(interpolendValue, low.getKey(), high.getKey(), low.getValue().getForce(), high.getValue().getForce());
        double interR = interpolateLinear(interpolendValue, low.getKey(), high.getKey(), low.getValue().getForceRotation().getRadians(), high.getValue().getForceRotation().getRadians());

        return BreakerVector2.fromForceAndRotation(new Rotation2d(interR), interF);
    }
}
