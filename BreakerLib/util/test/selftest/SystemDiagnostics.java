// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.selftest;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerCTREUtil;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerREVUtil;

/** A higher level object for use in user susystems that makes BreakerLib's {@link SelfTest} functionality to implament for subsystem-scale classes */
public class SystemDiagnostics extends BreakerSelfTestableBase {
    private List<BaseMotorController> ctreMotorControllers = new ArrayList<>();
    private List<BreakerSelfTestable> devices = new ArrayList<>();
    private List<CANSparkMax> sparks = new ArrayList<>();
    private Supplier<DeviceHealth> deviceHealthSupplier;
    private Supplier<String> faultStringSupplier;
    private boolean usesSuppliers = false;

    /** Creates a new SystemDiagnostics object
     * @param systemName The name of this {@link BreakerSelfTestableBase} implamentation
     */
    public SystemDiagnostics(String systemName) {
        deviceName = systemName;
        usesSuppliers = false;
    }
 
    /** Adds new {@link DeviceHealth} and String suppliers to this diagnostics device, overwrites any exisiting suppliers you have allready added
     * @param deviceHealthSupplier The {@link DeviceHealth} supplier to use (result considered with outher added factors)
     * @param faultStringSupplier The String supplier to use (result added to overall fault string)
     */
    public void addSuppliers(Supplier<DeviceHealth> deviceHealthSupplier, Supplier<String> faultStringSupplier) {
        this.deviceHealthSupplier = deviceHealthSupplier;
        this.faultStringSupplier = faultStringSupplier;
        usesSuppliers = true;
    }

    /** Adds a {@link BreakerSelfTestable} enabled object to this SystemDiagnostic's testing queue */
    public void addBreakerDevice(BreakerSelfTestable deviceToAdd) {
        devices.add(deviceToAdd);
    }

    /** Adds multipul {@link BreakerSelfTestable} enabled objects to this SystemDiagnostic's testing queue */
    public void addBreakerDevices(BreakerSelfTestable... devicesToAdd) {
        for (BreakerSelfTestable div: devicesToAdd) {
            devices.add(div);
        }
    }

    /** Adds a {@link BaseMotorController} (CTRE Motor controller) object to this SystemDiagnostic's testing queue */
    public void addCTREMotorController(BaseMotorController motorControllerToAdd) {
        ctreMotorControllers.add(motorControllerToAdd);
    }

    /** Adds multipul {@link BaseMotorController} (CTRE Motor controller) objects to this SystemDiagnostic's testing queue */
    public void addCTREMotorControllers(BaseMotorController... motorControllersToAdd) {
        for (BaseMotorController con: motorControllersToAdd) {
            addCTREMotorController(con);
        }
    }

    /** Adds a {@link CANSparkMax} (REV Motor controller) object to this SystemDiagnostic's testing queue */
    public void addSparkMax(CANSparkMax sparkMaxToAdd) {
        sparks.add(sparkMaxToAdd);
    }

    /** Adds multipul {@link CANSparkMax} (REV Motor controller) objects to this SystemDiagnostic's testing queue */
    public void addSparkMaxs(CANSparkMax... sparkMaxsToAdd) {
        for (CANSparkMax spk: sparkMaxsToAdd) {
            addSparkMax(spk);
        }
    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;

        if (!devices.isEmpty()) {
            for (BreakerSelfTestable div: devices) {
                div.runSelfTest();
                if (div.hasFault()) {
                    faultStr += " / " + div.getDeviceName() + ": " + div.getFaults();
                    health = health == DeviceHealth.INOPERABLE ? div.getHealth() : health;
                }
            }
        }

        if (!ctreMotorControllers.isEmpty()) {
            for (BaseMotorController con: ctreMotorControllers) {
                Faults motorFaults = new Faults();
                con.getFaults(motorFaults);
                if (motorFaults.hasAnyFault()) {
                    Pair<DeviceHealth, String> motorState = BreakerCTREUtil.getMotorHealthAndFaults(motorFaults);
                    faultStr += " / CTRE Motor ID (" + con.getBaseID() + "): " + motorState.getSecond();
                    health = health == DeviceHealth.INOPERABLE ? motorState.getFirst() : health;
                }
            }
        }

        if (!sparks.isEmpty()) {
            for (CANSparkMax spk: sparks) {
                Pair<DeviceHealth, String> sparkState = BreakerREVUtil.getSparkMaxHealthAndFaults(spk.getFaults());
                faultStr += " / SparkMax Motor ID (" + spk.getDeviceId() + "): " + sparkState.getSecond();
            }
        }

        if (usesSuppliers) {
            faultStr += " / Supplied Faults: " + faultStringSupplier.get();
            if (health != DeviceHealth.INOPERABLE && deviceHealthSupplier.get() != DeviceHealth.NOMINAL) {
                health = deviceHealthSupplier.get();
            }
        }
    }

}
