// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class BreakerQuadratic extends BreakerMathFunction {
    public BreakerQuadratic(double a, double b, double c) {
        super((Double x) -> (a * (x * x) + b * x + c));
    }

    public BreakerQuadratic() {
        super((Double x) -> (x * x));
    }
}
