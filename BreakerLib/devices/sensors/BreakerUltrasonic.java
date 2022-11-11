// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Ultrasonic;

/** Wrapper class for an {@link Ultrasonic}. */
public class BreakerUltrasonic {

    private DigitalOutput pingCh;
    private DigitalInput echoCh;
    private Ultrasonic sonic;
    private double offsetMM;

    /** Constructs an ultrasonic sensor. Automatic mode is enabled.
     * 
     * @param pingOutChannelDIO Channel for ping DIO.
     * @param echoInChannelDIO Channel for echo DIO.
     */
    public BreakerUltrasonic(int pingOutChannelDIO, int echoInChannelDIO) {
        sonic = new Ultrasonic(pingOutChannelDIO, echoInChannelDIO); //new Ultrasonic(pingCh, echoCh);
        Ultrasonic.setAutomaticMode(true);
    }

    /** Constructs an ultrasonic sensor with an offset. Automatic mode is enabled.
     * 
     * @param pingOutChannelDIO Channel for ping DIO.
     * @param echoInChannelDIO Channel for echo DIO.
     * @param offsetMM Distance offset in mm.
     */
    public BreakerUltrasonic(int pingOutChannelDIO, int echoInChannelDIO, double offsetMM) {
        this.offsetMM = offsetMM;
        sonic = new Ultrasonic(pingCh, echoCh);
        Ultrasonic.setAutomaticMode(true);
    }

    /** @return Detected distance in mm. */
    public double getRangeMM() {
        return sonic.getRangeMM() - offsetMM;
    }

    /** @return Detected distance in inches. */
    public double getRangeInches() {
        return sonic.getRangeInches() - Units.metersToInches(offsetMM/1000);
    }
}
