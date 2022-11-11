// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.rotation;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.DoubleAccumulator;
import java.util.function.Function;
import java.util.function.Supplier;

import javax.print.Doc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps.BreakerInterpolatingTreeMap;

/** A class that supplyes a swerve drive trajectory follower with live rotation setpoint targets */
public class BreakerSwerveRotationSupplier implements BreakerGenericSwerveRotationSupplier {
    private BreakerRotationPoint[] rotationPoints;
    private Function<Double, Rotation2d> externalFunction;
    private BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> interMap;
    private boolean  usesFunc;
    /** Creates a new {@link BreakerSwerveRotationSupplier} with a supplied rotation that defaults to 0 degrees
    */
    public BreakerSwerveRotationSupplier() {
        usesFunc = true;
        externalFunction = (Double curTime) -> new Rotation2d();
    }

    /** Creates a new {@link BreakerSwerveRotationSupplier} with an array of time deifined rotation points that will be interpolated 
     * @param rotationPoints Time defigned rotations to interpolate between
    */
    public BreakerSwerveRotationSupplier(BreakerRotationPoint... rotationPoints) {
        this.rotationPoints = rotationPoints;
        usesFunc = false;
        for (BreakerRotationPoint point: rotationPoints) {
            interMap.put(point.getTimeOfRotation(), new BreakerInterpolableDouble(point.getRotation().getRadians()));
        }
    }

    /** Creates a new {@link BreakerSwerveRotationSupplier} with a List of time deifined rotation points that will be interpolated 
     * @param rotationPoints List of time defigned rotations to interpolate between
    */
    public BreakerSwerveRotationSupplier(List<BreakerRotationPoint> rotationPoints) {
        this.rotationPoints = rotationPoints.toArray(new BreakerRotationPoint[rotationPoints.size()]);
        usesFunc = false;
        for (BreakerRotationPoint point: rotationPoints) {
            interMap.put(point.getTimeOfRotation(), new BreakerInterpolableDouble(point.getRotation().getRadians()));
        }
    }

    /** Creates a new {@link BreakerSwerveRotationSupplier} with a supplier for the current rotation setpoint
     * @param rotationPoints A supplier for the current {@link Rotation2d} setpoint
    */
    public BreakerSwerveRotationSupplier(Supplier<Rotation2d> externalSupplier) {
        this.externalFunction = (Double time) -> (externalSupplier.get());
        usesFunc = true;
    }

    /** Creates a new {@link BreakerSwerveRotationSupplier} with a Function that takes in an argument 
     * of the current elapsed path time in seconds and returns the {@link Rotation2d} setpoint
     * @param rotationPoints The Function that supplies the curent rotation setpoint
    */
    public BreakerSwerveRotationSupplier(Function<Double, Rotation2d> externalFunction) {
        this.externalFunction = externalFunction;
        usesFunc = true;
    }

    @Override
    public Rotation2d getRotation(double currentTime) {
        if (usesFunc) {
            return externalFunction.apply(currentTime);
        }
        return new Rotation2d(interMap.getInterpolatedValue(currentTime).getValue());
    }

}
