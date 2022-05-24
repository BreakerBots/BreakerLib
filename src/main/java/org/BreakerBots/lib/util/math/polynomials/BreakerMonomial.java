// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.polynomials;

/** Add your docs here. */
public class BreakerMonomial {
    private double coeficent = 1;
    private double degree = 0;
    private double baseNumber = 0;
    public BreakerMonomial(double coeficent, double degree) {
        this.coeficent = coeficent;
        this.degree = degree;
    }
    
    public BreakerMonomial(double coeficent, double baseNumber, double degree) {
        this.coeficent = coeficent;
        this.degree = degree;
        this.baseNumber = baseNumber;
    }

    public BreakerMonomial(double baseNumber) {
        this.baseNumber = baseNumber;
    }

    public double getValueAtX(double xValue) {
        return coeficent * (Math.pow(xValue, degree) + baseNumber);
    }

    public BreakerMonomial multiply(BreakerMonomial outher) {
        return new BreakerMonomial(coeficent * outher.getCoeficent(), baseNumber * outher.getBaseNumber(), degree + outher.getDegree());
    }

    public double getCoeficent() {
        return coeficent;
    }

    public double getDegree() {
        return degree;
    }

    public double getBaseNumber() {
        return baseNumber;
    }
}
