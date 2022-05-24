// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

/** Add your docs here. */
public class BreakerCANdleConfig {
    private CANdleConfiguration config;
    public BreakerCANdleConfig(double brightnessScalar, LEDStripType stripType, VBatOutputMode vBatOutputMode, 
        boolean disableOnSignalLoss, boolean enableOptmizations, boolean statusLEDOffWhenActive) {
        config = new CANdleConfiguration();
        config.brightnessScalar = brightnessScalar;
        config.stripType = stripType;
        config.vBatOutputMode = vBatOutputMode;
        config.disableWhenLOS = disableOnSignalLoss;
        config.enableOptimizations = enableOptmizations;
        config.statusLedOffWhenActive = statusLEDOffWhenActive;
    }

    public CANdleConfiguration getConfig() {
        return config;
    }
}
