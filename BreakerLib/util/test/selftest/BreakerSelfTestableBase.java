// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.selftest;

/** A higher level implamentation of the {@link BreakerSelfTestable} interface */
public abstract class BreakerSelfTestableBase implements BreakerSelfTestable  {
    protected String faultStr = null, deviceName = " Unknown_Device ";
    protected DeviceHealth health = DeviceHealth.NOMINAL;

    @Override
    public boolean hasFault() {
        return health != DeviceHealth.NOMINAL;
    }

    @Override
    public DeviceHealth getHealth() {
        return health;
    }

    @Override
    public String getFaults() {
        return faultStr;
    }

    @Override
    public String getDeviceName() {
        return deviceName;
    }

    @Override
    public void setDeviceName(String newName) {
        deviceName = newName;
    }

    
}
