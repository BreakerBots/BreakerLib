// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.selftest;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.databind.JsonSerializable.Base;

import edu.wpi.first.cscore.CameraServerCvJNI.Helper;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.devices.cosmetic.led.BreakerCANdle.BreakerCANdleLedMode;
import frc.robot.BreakerLib.util.BreakerCTREUtil;
import frc.robot.BreakerLib.util.BreakerTriplet;

/** a higher level object for use in user susystems that makes BreakerLib's self test clases easier to implament for subsystem-scale classes */
public class SystemDiagnostics implements BreakerSelfTestable {
    private List<BaseMotorController> motorControllers = new ArrayList<>();
    private List<BreakerSelfTestable> devices = new ArrayList<>();
    private List<Integer> outherIdsCAN = new ArrayList<>();
    private Supplier<DeviceHealth> deviceHealthSupplier;
    private Supplier<String> faultStringSupplier;
    private boolean usesSuppliers = false;
    private String faults;
    private String systemName;
    private DeviceHealth health = DeviceHealth.NOMINAL;
    public SystemDiagnostics(String systemName) {
        SelfTest.addDevice(this);
        this.systemName = systemName;
        usesSuppliers = false;
    }

    public SystemDiagnostics(String systemName, Supplier<DeviceHealth> deviceHealthSupplier, Supplier<String> faultStringSupplier) {
        SelfTest.addDevice(this);
        this.deviceHealthSupplier = deviceHealthSupplier;
        this.faultStringSupplier = faultStringSupplier;
        this.systemName = systemName;
        usesSuppliers = true;
    }

    public void addBreakerDevice(BreakerSelfTestable deviceToAdd) {
        devices.add(deviceToAdd);
    }

    public void addBreakerDevices(BreakerSelfTestable... devicesToAdd) {
        for (BreakerSelfTestable div: devicesToAdd) {
            devices.add(div);
        }
    }

    public void addMotorController(BaseMotorController motorControllerToAdd) {
        motorControllers.add(motorControllerToAdd);
    }

    public void addMotorControllers(BaseMotorController... motorControllersToAdd) {
        for (BaseMotorController con: motorControllersToAdd) {
            addMotorController(con);
        }
    }

    public void addOutherGenericCANDevice(int deviceID) {
        outherIdsCAN.add(deviceID);
    }

    public void addOutherGenericCANDevices(int... deviceIDs) {
        for (int id: deviceIDs) {
            outherIdsCAN.add(id);
        }
    }    

    @Override
    public void runSelfTest() {
        faults = null;
        health = DeviceHealth.NOMINAL;
        if (!devices.isEmpty()) {
            for (BreakerSelfTestable div: devices) {
                div.runSelfTest();
                if (div.hasFault()) {
                    faults += " / " + div.getDeviceName() + ": " + div.getFaults();
                    health = health == DeviceHealth.INOPERABLE ? div.getHealth() : health;
                }
            }
        }
        if (!motorControllers.isEmpty()) {
            for (BaseMotorController con: motorControllers) {
                Faults motorFaults = new Faults();
                con.getFaults(motorFaults);
                if (motorFaults.hasAnyFault()) {
                    BreakerTriplet<DeviceHealth, String, Boolean> motorState = BreakerCTREUtil.getMotorHealthFaultsAndConnectionStatus(motorFaults, con.getDeviceID());
                    faults += " / Motor ID (" + con.getBaseID() + "): " + motorState.getMiddle();
                    health = health == DeviceHealth.INOPERABLE ? motorState.getLeft() : health;
                }
            }
        }
        if (!outherIdsCAN.isEmpty()) {
            for (int id: outherIdsCAN) {
                if (SelfTest.checkIsMissingCanID(id)) {
                    faults += " / Generic CAN device (ID: " + id + "): not_found_on_buss ";
                    health = DeviceHealth.INOPERABLE;
                }
            }
        }
        if (usesSuppliers) {
            faults += " / Supplied Faults: " + faultStringSupplier.get();
        }
    }

    @Override
    public DeviceHealth getHealth() {
        if (health != DeviceHealth.INOPERABLE && usesSuppliers) {
            return deviceHealthSupplier.get();
        }
        return health;
    }

    @Override
    public String getFaults() {
        return faults;
    }

    @Override
    public String getDeviceName() {
        return systemName;
    }

    @Override
    public boolean hasFault() {
        return getHealth() != DeviceHealth.NOMINAL;
    }

    @Override
    public void setDeviceName(String newName) {
        systemName = newName;
    }
}
