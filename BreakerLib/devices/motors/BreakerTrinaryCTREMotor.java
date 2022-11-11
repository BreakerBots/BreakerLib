// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.devices.BreakerGenericDeviceBase;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerCTREUtil;

/** Falcon motor with simple forward/reverse/off controls */
public class BreakerTrinaryCTREMotor extends BreakerGenericDeviceBase {

    private BaseMotorController motor;
    private double forwardOut, reverseOut;

    /**
     * Create a new TrinaryCTREMotor with 100% forward input and -100% reverse input.
     * 
     * @param motor CTRE motor controller.
     */
    public BreakerTrinaryCTREMotor(BaseMotorController motor) {
        new BreakerTrinaryCTREMotor(motor, 1, -1);
    }

    /**
     * Create a new TrinaryCTREMotor that has given output % forward, negative given output % reverse.
     * 
     * @param motor  CTRE motor controller.
     * @param output Percent output between -1 and 1.
     */
    public BreakerTrinaryCTREMotor(BaseMotorController motor, double output) {
        new BreakerTrinaryCTREMotor(motor, output, -output);
    }

    /**
     * Create a new TrinaryCTREMotor that has separate forward output % and reverse output %
     * 
     * @param motor CTRE motor controller.
     * @param forwardOut Forward % output between -1 and 1.
     * @param reverseOut Reverse % output between -1 and 1.
     */
    public BreakerTrinaryCTREMotor(BaseMotorController motor, double forwardOut, double reverseOut) {
        this.motor = motor;
        this.forwardOut = forwardOut;
        this.reverseOut = reverseOut;
        deviceName = " Trinary_Motor (" + motor.getDeviceID() + ") ";
    }

    /** Sets motor to designated forward percent output. */
    public void forward() {
        motor.set(ControlMode.PercentOutput, forwardOut);
    }

    /** Sets motor to designated reverse percent output. */
    public void reverse() {
        motor.set(ControlMode.PercentOutput, reverseOut);
    }

    /** Sets motor to 0% output (stopped) */
    public void stop() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    /** Checks if motor is running or not. */
    public boolean isActive() {
        return (motor.getMotorOutputPercent() != 0);
    }

    /** @return Base CTRE motor controller. */
    public BaseMotorController getMotor() {
        return motor;
    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;
        Faults faultObj = new Faults();
        motor.getFaults(faultObj);
        if (faultObj.hasAnyFault()) {
            Pair<DeviceHealth, String> pair = BreakerCTREUtil
                    .getMotorHealthAndFaults(faultObj);
            faultStr = pair.getSecond();
            health = pair.getFirst();
        }
    }

    @Override
    public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
        // TODO Auto-generated method stub

    }

    @Override
    public void returnToAutomaticPowerManagement() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isUnderAutomaticControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public DevicePowerMode getPowerMode() {
        // TODO Auto-generated method stub
        return null;
    }

}
