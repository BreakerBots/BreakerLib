// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.pneumatics;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * Compressor with built-in AnalogPotentiometer and pneumatic module support.
 */
public class BreakerCompressor {

    private PneumaticsModuleType moduleType;

    private PneumaticsBase pneumaticModule;
    private AnalogPotentiometer analogPressureSensor = new AnalogPotentiometer(999); // Basically a null pressure
                                                                                     // sensor.

    /** Creates a new BreakerCompressor. */
    public BreakerCompressor(int moduleID, PneumaticsModuleType moduleType) {
        this.moduleType = moduleType;
        moduleSetup(moduleID);
    }

    /**
     * Creates a new BreakerCompressor with default module ID (0 for PCM, 1 for PH).
     */
    public BreakerCompressor(PneumaticsModuleType moduleType) {
        this.moduleType = moduleType;
        moduleSetup();
    }

    /** Creates PCM or PH in constructor. */
    private void moduleSetup(int id) {
        switch (moduleType) {
            case CTREPCM:
                pneumaticModule = new PneumaticsControlModule(id);
                break;
            case REVPH:
                pneumaticModule = new PneumaticHub(id);
                break;
        }
    }

    /** Creates PCM or PH in constructor. */
    private void moduleSetup() {
        switch (moduleType) {
            case CTREPCM:
                pneumaticModule = new PneumaticsControlModule();
                break;
            case REVPH:
                pneumaticModule = new PneumaticHub();
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

}
