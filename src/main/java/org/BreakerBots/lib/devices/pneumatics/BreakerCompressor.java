// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.pneumatics;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Compressor with built-in AnalogPotentiometer and pneumatic module support. */
public class BreakerCompressor {

    
    private PneumaticsModuleType moduleType;
    private int moduleID;

    private AnalogPotentiometer analogPressureSensor;
    private Compressor airCompressor;

    public BreakerCompressor(int moduleID, PneumaticsModuleType moduleType) {
        this.moduleType = moduleType;
        this.moduleID = moduleID;
        airCompressor = new Compressor(moduleID, moduleType);
    }

    public BreakerCompressor(PneumaticsModuleType type) {
        moduleType = type;
        airCompressor = new Compressor(moduleType);
    }

    private void moduleSetup(int id) {

    }
    
}
