// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

/** Add your docs here. */
public class BreakerPolynomial extends BreakerMathFunction {
    public BreakerPolynomial(BreakerMonomial...monomials) {
        super((Double x) -> {Double total = 0.0; for (BreakerMonomial mon: monomials) {total += mon.getValueAtX(x);} return total;});
        
    }
}
