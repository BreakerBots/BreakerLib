// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

/** A interface for all BreakerLib classes that represent pure mathmatical functions (or equations) such as {@link BreakerBezierCurve} or {@link BreakerQuadratic}*/
public interface BreakerGenericMathFunction {
    /** @param xValue the value of "x" or "t" to check
     * @return the function's value at the given point
     */
    public abstract double getValueAtX(double xValue);

    /** @param xValue the value of "x" or "t" to check
     * @return f(abs(x)) * the sign of x
     */
    public default double getSignRelativeValueAtX(double xValue) {
        return getValueAtX(Math.abs(xValue)) * (xValue < 0 ? -1 : 1);
    }

    /** @return a new fuction representing the algabraic sum of this function and the given function */
    public abstract BreakerGenericMathFunction add(BreakerGenericMathFunction funcToAdd);

    /** @return a new fuction representing the algabraic sum of this function and the given function negated */
    public abstract BreakerGenericMathFunction subtract(BreakerGenericMathFunction funcToSubtract);

    /** @return a new fuction representing the algabraic product of this function and the given function */
    public abstract BreakerGenericMathFunction multiply(BreakerGenericMathFunction multipul);

     /** @return a new fuction representing the algabraic quotent of this function and the given function */
    public abstract BreakerGenericMathFunction devide(BreakerGenericMathFunction dividend);
}
