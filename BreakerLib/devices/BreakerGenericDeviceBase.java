// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices;

import frc.robot.BreakerLib.util.logging.BreakerGenericTelemetryProvider;
import frc.robot.BreakerLib.util.power.BreakerPowerManageable;
import frc.robot.BreakerLib.util.test.selftest.BreakerSelfTestableBase;
import frc.robot.BreakerLib.util.test.selftest.SelfTest;

/** Less complcated alternative to BreakerGenericDevice that also automaticly regesters Devices */
public abstract class BreakerGenericDeviceBase extends BreakerSelfTestableBase implements BreakerPowerManageable {
    public BreakerGenericDeviceBase() {
        SelfTest.autoRegisterDevice(this);
    }

}
