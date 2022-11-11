// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.functions;

import org.ejml.equation.Function;

import edu.wpi.first.math.geometry.Translation2d;

/** A controllable bezier spline: https://www.desmos.com/calculator/da8zwxpgzo */
public class BreakerBezierCurve extends BreakerMathFunction {
  
    /** creates a new cubic bezier spline with two control points. (Start(0,0), end(1,1)) */
    public BreakerBezierCurve(Translation2d controlPointOne, Translation2d controlPointTwo) {
        super((Double x) -> {
          double x0 = 0.0;
          double y0 = 0.0;
          double x1 = controlPointOne.getX();
          double y1 = controlPointOne.getY();
          double x2 = controlPointTwo.getX();
          double y2 = controlPointTwo.getY();
          double x3 = 1.0;
          double y3 = 1.0;
          x = (1 - x);
          x = 1 - (((1 - x) * p(x, x0, x1, x2)) + (x * p(x, x1, x2, x3)));
          x = (((1 - x) * p(x, y0, y1, y2)) + (x * p(x, y1, y2, y3)));
          return x;
        });
    }

    /** creates a new cubic bezier spline with all four control points available */
    public BreakerBezierCurve(Translation2d startPoint, Translation2d controlPointOne, Translation2d controlPointTwo, Translation2d endPoint) {
        super((Double x) -> {
          double x0 = startPoint.getX();
          double y0 = startPoint.getY();
          double x1 = controlPointOne.getX();
          double y1 = controlPointOne.getY();
          double x2 = controlPointTwo.getX();
          double y2 = controlPointTwo.getY();
          double x3 = endPoint.getX();
          double y3 = endPoint.getY();
          x = (1 - x);
          x = 1 - (((1 - x) * p(x, x0, x1, x2)) + (x * p(x, x1, x2, x3)));
          x = (((1 - x) * p(x, y0, y1, y2)) + (x * p(x, y1, y2, y3)));
          return x;
        });
    }
  
    private static double p(double x, double a, double b, double c) {
      return ((1 - x) * ((1 - x) * a + (x * b))) + (x * ((1 - x) * b + (x * c)));
    }

}
