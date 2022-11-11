// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.accelerometer;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;

/** RoboRIO accelerometer using the {@link BreakerGenericAccelerometer} interface. */
public class RoboRIOAccelerometer implements BreakerGenericAccelerometer{

    BuiltInAccelerometer accelerometer;
    
    /** RoboRIO accelerometer with range of 8 Gs. */
    public RoboRIOAccelerometer() {
        accelerometer = new BuiltInAccelerometer();
    }

    /** RoboRIO accelerometer with specified range. */
    public RoboRIOAccelerometer(Range range) {
        accelerometer = new BuiltInAccelerometer(range);
    }

    @Override
    public double[] getRawAccelerometerVals() {
        return new double[] {accelerometer.getX(), accelerometer.getY(), accelerometer.getZ()};
    }

    @Override
    public double getRawAccelX() {
        return accelerometer.getX();
    }

    @Override
    public double getRawAccelY() {
        return accelerometer.getY();
    }

    @Override
    public double getRawAccelZ() {
        return accelerometer.getZ();
    }

    @Override
    public void setRange(Range range) {
        accelerometer.setRange(range);
    }}
