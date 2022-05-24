// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

/** Add your docs here. */
public class BreakerInterpolateable {
    
    public double interpolateLinear(double knownX, double lowX, double highX, double lowY, double highY) {
        return (((knownX - lowX) * (highY - lowY)) / (highX - lowX)) + lowY;
    }


}
