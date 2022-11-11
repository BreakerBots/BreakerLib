// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

import java.util.function.Function;

/** Add your docs here. */
public class BreakerExponential extends BreakerMathFunction {
    /** a(b^(x+c))+d */
    public BreakerExponential(double a, double b, double c, double d) {
        super((Double x) -> ((a * Math.pow(b, x + c)) + d));
    }

}
