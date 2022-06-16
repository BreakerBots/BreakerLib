 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices;

import frc.robot.BreakerLib.util.powermanagement.BreakerPowerManageable;
import frc.robot.BreakerLib.util.selftest.BreakerSelfTestable;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;

/** Base interface for all Breaker devices. */
public interface BreakerGenericDevice extends BreakerSelfTestable, BreakerPowerManageable {
}
