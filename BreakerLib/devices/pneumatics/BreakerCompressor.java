// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.pneumatics;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.BreakerLib.devices.BreakerGenericDeviceBase;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;

/**
 * Compressor with built-in AnalogPotentiometer and pneumatic module support.
 */
public class BreakerCompressor extends BreakerGenericDeviceBase {

    private PneumaticsModuleType moduleType;

    private PneumaticsBase pneumaticModule;
    private PneumaticsControlModule ctrePCM;
    private PneumaticHub revPneumaticHub;
    private AnalogPotentiometer analogPressureSensor = new AnalogPotentiometer(0); // Basically a null pressure
                                                                                   // sensor until instantiated.

    /** Creates a new BreakerCompressor. with provided ID and type.
     * 
     * @param moduleID CAN ID for the module.
     * @param moduleType CTRE or REV.
    */
    public BreakerCompressor(int moduleID, PneumaticsModuleType moduleType) {
        this.moduleType = moduleType;
        deviceName = "Pnumatics_Module";
        moduleSetup(moduleID);
    }

    /**
     * Creates a new BreakerCompressor with default module ID (0 for PCM, 1 for PH).
     * 
     * @param moduleType CTRE or REV.
     */
    public BreakerCompressor(PneumaticsModuleType moduleType) {
        this.moduleType = moduleType;
        deviceName = "Pnumatics_Module";
        moduleSetup();
    }

    /** Creates PCM or PH in constructor. */
    private void moduleSetup(int id) {
        switch (moduleType) {
            case CTREPCM:
                ctrePCM = new PneumaticsControlModule(id); // Registered as spesific model for self test
                pneumaticModule = ctrePCM;
                break;
            case REVPH:
                revPneumaticHub = new PneumaticHub(id); // Registered as spesific model for self test
                pneumaticModule = revPneumaticHub;
                break;
        }
    }

    /** Creates PCM or PH in constructor. */
    private void moduleSetup() {
        switch (moduleType) {
            case CTREPCM:
                ctrePCM = new PneumaticsControlModule(); // Registered as spesific model for self test
                pneumaticModule = ctrePCM;
                break;
            case REVPH:
                revPneumaticHub = new PneumaticHub(); // Registered as spesific model for self test
                pneumaticModule = revPneumaticHub;
                break;
        }
    }

    /**
     * Creates an analog pressure sensor based on the REV Analog Pressure Sensor.
     * <p>
     * Only need to do this if using PCM. Otherwise just plug the sensor into analog
     * port 0 on the PH.
     */
    public void addAnalogPressureSensor(int analog_channel) {
        analogPressureSensor = new AnalogPotentiometer(analog_channel, 250, -25);
    }

    /**
     * Creates an analog pressure sensor with given full range and offset.
     * <p>
     * Only need to do this if using PCM. Otherwise just plug the sensor into analog
     * port 0 on the PH.
     * 
     * @param analog_channel Analog port the sensor is plugged into.
     * @param fullRange      Scaling multiplier to output (check with part
     *                       manufacturer).
     * @param offset         Offset added to scaled value.
     */
    public void addAnalogPressureSensor(int analog_channel, double fullRange, double offset) {
        analogPressureSensor = new AnalogPotentiometer(analog_channel, fullRange, offset);
    }

    /**
     * Returns psi measured by analog pressure sensor. If no pressure sensor,
     * returns 0.
     */
    public double getPressure() {
        double pressure = pneumaticModule.getPressure(0);
        if (pressure != 0) {
            return pressure;
        } else {
            pressure = analogPressureSensor.get();
        }
        return pressure;
    }

    /** Returns voltage of analog port 0 if supported. */
    public double getVoltage() {
        return pneumaticModule.getAnalogVoltage(0);
    }

    /** Current in amps used by the compressor */
    public double getCompressorAmps() {
        return pneumaticModule.getCompressorCurrent();
    }

    /** Returns base pneumatic module. */
    public PneumaticsBase getModule() {
        return pneumaticModule;
    }

    /** Returns true if the compressor is enabled. */
    public boolean compressorIsEnabled() {
        return pneumaticModule.getCompressor();
    }

    /** Enables closed loop compressor control using digital input. */
    public void enableDigital() {
        pneumaticModule.enableCompressorDigital();
    }

    /**
     * Enables closed loop compressor control using analog input from REV Analog
     * Pressure Sensor only. Defaults to digital control if CTRE PCM.
     * 
     * @param minPressure Compressor enables when pressure is below this value.
     * @param maxPressure Compressor disables when pressure is above this value.
     */
    public void enableAnalog(double minPressure, double maxPressure) {
        pneumaticModule.enableCompressorAnalog(minPressure, maxPressure);
    }

    /**
     * Enables closed loop compressor control using hybrid input from REV Analog
     * Pressure Sensor only. Defaults to digital control if CTRE PCM.
     * 
     * @param minPressure Compressor enables when pressure is below this value.
     * @param maxPressure Compressor disables when pressure is above this value.
     */
    public void enableHybrid(double minPressure, double maxPressure) {
        pneumaticModule.enableCompressorHybrid(minPressure, maxPressure);
    }

    /** Disables the compressor, shutting it down. */
    public void disable() {
        pneumaticModule.disableCompressor();
    }

    @Override
    public void runSelfTest() {
        // WHY?!
        switch (moduleType) {
            case CTREPCM:
                break;
            case REVPH:
                break;

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
