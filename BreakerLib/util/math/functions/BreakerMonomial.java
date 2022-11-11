// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

/** Add your docs here. */
public class BreakerMonomial extends BreakerMathFunction {
    private double coeficent = 1;
    private double degree = 0;
    
    public BreakerMonomial(double constant) {
        super((Double x) -> (constant));
        coeficent = constant;
        degree = 0;
    }
    
    public BreakerMonomial(double coeficent, double degree) {
        super((Double x) -> (coeficent * Math.pow(x, degree)));
        this.coeficent = coeficent;
        this.degree = degree;
    }

    public double getCoeficent() {
        return coeficent;
    }

    public double getDegree() {
        return degree;
    }
}
