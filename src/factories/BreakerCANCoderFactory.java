// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.factories;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.robot.BreakerLib.util.BreakerCTREUtil;

/** Add your docs here. */
public class BreakerCANCoderFactory {
    public static WPI_CANCoder createCANCoder(int deviceID, AbsoluteSensorRange absoluteSensorRange, double absoluteOffsetDegress, boolean invertEncoder) {
        WPI_CANCoder encoder = new WPI_CANCoder(deviceID);
        CANCoderConfiguration config = new CANCoderConfiguration();
            config.absoluteSensorRange = absoluteSensorRange;
            config.magnetOffsetDegrees = absoluteOffsetDegress;
            config.sensorDirection = invertEncoder;
        BreakerCTREUtil.checkError(encoder.configAllSettings(config, 0), " CANCoder factory config fail ");
        return encoder;
    }
}
