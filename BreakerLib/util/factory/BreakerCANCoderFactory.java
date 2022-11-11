// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.factory;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.robot.BreakerLib.util.test.vendorutil.BreakerCTREUtil;

/** Factory for producing CANcoders. */
public class BreakerCANCoderFactory {
    public static WPI_CANCoder createCANCoder(int deviceID, SensorInitializationStrategy initializationStrategy, AbsoluteSensorRange absoluteSensorRange, double absoluteOffsetDegress, boolean encoderDirection) {
        WPI_CANCoder encoder = new WPI_CANCoder(deviceID);
        configExistingCANCoder(encoder, initializationStrategy, absoluteSensorRange, absoluteOffsetDegress, encoderDirection);
        return encoder;
    }

    public static WPI_CANCoder createCANCoder( int deviceID, String busName, SensorInitializationStrategy initializationStrategy, AbsoluteSensorRange absoluteSensorRange, double absoluteOffsetDegress, boolean encoderDirection) {
        WPI_CANCoder encoder = new WPI_CANCoder(deviceID, busName);
        configExistingCANCoder(encoder, initializationStrategy, absoluteSensorRange, absoluteOffsetDegress, encoderDirection);
        return encoder;
    }

    public static void configExistingCANCoder(WPI_CANCoder encoder, SensorInitializationStrategy initializationStrategy, AbsoluteSensorRange absoluteSensorRange, double absoluteOffsetDegress, boolean encoderDirection) {
        BreakerCTREUtil.checkError(encoder.configAbsoluteSensorRange(absoluteSensorRange), " CANCoder " + encoder.getDeviceID() + " ABS sensor range config fail ");
        BreakerCTREUtil.checkError(encoder.configSensorInitializationStrategy(initializationStrategy), " CANCoder " + encoder.getDeviceID() + " init stratagy config fail ");
        BreakerCTREUtil.checkError(encoder.configMagnetOffset(absoluteOffsetDegress), " CANCoder " + encoder.getDeviceID() + " mag offest angle config fail ");
        BreakerCTREUtil.checkError(encoder.configSensorDirection(encoderDirection), " CANCoder " + encoder.getDeviceID() + " sensor direction config fail ");
    }
}
