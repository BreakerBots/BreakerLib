// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerGenericInterpolatingMap;

/**
 * A class that acts as a provider for arbitrary feedforward demand values used
 * with the Talon or SparkMax motor controller's integrated PID.
 */
public class BreakerArbitraryFeedforwardProvider {

    private BreakerGenericInterpolatingMap<Double, Double> ffMap;
    private double feedforwardCoeficent, staticFrictionCoeficent;
    private Function<Double, Double> ffFunc;
    private SimpleMotorFeedforward ffClass;

    private FeedForwardType ffType;

    /** Type of arbitrary feedforward provider. */
    private enum FeedForwardType {
        MAP_SUP,
        COEFS,
        FUNC,
        FF_CLASS
    }

    /**
     * Creates a map-based ArbitraryFeedForwardProvider.
     * 
     * @param speedToFeedforwardValMap
     */
    public BreakerArbitraryFeedforwardProvider(
            BreakerGenericInterpolatingMap<Double, Double> speedToFeedforwardValMap) {
        ffMap = speedToFeedforwardValMap;
        ffType = FeedForwardType.MAP_SUP;
    }

    /**
     * Creates a coefficient-based ArbitraryFeedForwardProvider.
     * 
     * @param feedforwardCoefficient    Feedforward kV coefficient.
     * @param staticFrictionCoefficient Feedforward kS coefficient.
     */
    public BreakerArbitraryFeedforwardProvider(double feedforwardCoefficient, double staticFrictionCoefficient) {
        this.feedforwardCoeficent = feedforwardCoefficient;
        this.staticFrictionCoeficent = staticFrictionCoefficient;
        ffType = FeedForwardType.COEFS;
    }

    /**
     * Creates a function-based ArbitraryFeedForwardProvider that takes input.
     * 
     * @param ffFunc Function that applies feedforward calculations to given
     *               velocity (m/s).
     */
    public BreakerArbitraryFeedforwardProvider(Function<Double, Double> ffFunc) {
        this.ffFunc = ffFunc;
        ffType = FeedForwardType.FUNC;
    }

    /**
     * Creates a function-based ArbitraryFeedForwardProvider that takes no input.
     * 
     * @param ffSupplier Supplier function that provides feedforward results.
     */
    public BreakerArbitraryFeedforwardProvider(DoubleSupplier ffSupplier) {
        ffFunc = (Double x) -> (ffSupplier.getAsDouble());
        ffType = FeedForwardType.FUNC;
    }

    /**
     * Creates an ArbitraryFeedForwardProvider with a standard feedforward object.
     * 
     * @param ffClass Feedforward object.
     */
    public BreakerArbitraryFeedforwardProvider(SimpleMotorFeedforward ffClass) {
        this.ffClass = ffClass;
        ffType = FeedForwardType.FF_CLASS;
    }

    /** @return Percent output to be added to the desired motors's output to achieve the desired speed. */
    public double getArbitraryFeedforwardValue(double curSpeed) {
        double feedForward = 0.0;
        switch (ffType) {
            case COEFS:
                feedForward = (feedforwardCoeficent * curSpeed + staticFrictionCoeficent)
                        / RobotController.getBatteryVoltage();
                break;
            case MAP_SUP:
                feedForward = ffMap.getInterpolatedValue(curSpeed);
                break;
            case FUNC:
                feedForward = ffFunc.apply(curSpeed);
                break;
            case FF_CLASS:
                feedForward = ffClass.calculate(curSpeed) / RobotController.getBatteryVoltage();
        }
        return feedForward;
    }
}
