// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

import java.util.function.Function;

/** A base implamentation of the {@link BreakerGenericMathFunction} interface as wrapper around the 
 * java.util.function.Function functional interface */
public class BreakerMathFunction implements BreakerGenericMathFunction {
    private Function<Double, Double> mathFunction;
    public BreakerMathFunction(Function<Double, Double> mathFunction) {
        this.mathFunction = mathFunction;
    }

    @Override
    public double getValueAtX(double xValue) {
        return mathFunction.apply(xValue);
    }

    @Override
    public BreakerGenericMathFunction add(BreakerGenericMathFunction funcToAdd) {
        return new BreakerMathFunction((Double x) -> (mathFunction.apply(x) + funcToAdd.getValueAtX(x)));
    }

    @Override
    public BreakerGenericMathFunction subtract(BreakerGenericMathFunction funcToSubtract) {
        return new BreakerMathFunction((Double x) -> (mathFunction.apply(x) - funcToSubtract.getValueAtX(x)));
    }

    @Override
    public BreakerGenericMathFunction multiply(BreakerGenericMathFunction multipul) {
        return new BreakerMathFunction((Double x) -> (mathFunction.apply(x) * multipul.getValueAtX(x)));
    }

    @Override
    public BreakerGenericMathFunction devide(BreakerGenericMathFunction divisor) {
        return new BreakerMathFunction((Double x) -> (mathFunction.apply(x) / divisor.getValueAtX(x)));
    }
    
}
