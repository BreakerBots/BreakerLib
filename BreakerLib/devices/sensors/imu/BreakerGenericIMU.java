// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors.imu;

import frc.robot.BreakerLib.devices.BreakerGenericDeviceBase;
import frc.robot.BreakerLib.devices.sensors.accelerometer.BreakerGenericAccelerometer;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGeneric3AxisGyro;

/** Generic IMU interface featuring both accelerometer and gyro functionality. */
public abstract class BreakerGenericIMU extends BreakerGenericDeviceBase implements BreakerGeneric3AxisGyro, BreakerGenericAccelerometer {}
