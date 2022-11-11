// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.pneumatics;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/** Solenoid with support for default state. */
public class BreakerSolenoid {
    private Solenoid solenoid;
    private boolean defaultState = false;

    /**
     * Constructs a BreakerDoubleSolenoid.
     * 
     * @param moduleType Module type used (CTRE or REV).
     * @param channel    Solenoid channel.
     */
    public BreakerSolenoid(PneumaticsModuleType moduleType, int channel) {
        solenoid = new Solenoid(moduleType, channel);
        toDefaultState();
    }

    /**
     * Constructs a BreakerDoubleSolenoid.
     * 
     * @param moduleType Module type used (CTRE or REV).
     * @param channel    Solenoid channel.
     * @param pcmID      ID for PCM.
     */
    public BreakerSolenoid(PneumaticsModuleType moduleType, int channel, int pcmID) {
        solenoid = new Solenoid(pcmID, moduleType, channel);
        toDefaultState();
    }

    /**
     * Constructs a BreakerDoubleSolenoid.
     * 
     * @param moduleType   Module type used (CTRE or REV).
     * @param channel      Solenoid channel.
     * @param defaultState Default solenoid state.
     */
    public BreakerSolenoid(PneumaticsModuleType moduleType, int channel, boolean defaultState) {
        solenoid = new Solenoid(moduleType, channel);
        setDefaultState(defaultState);
        toDefaultState();
    }

    /**
     * Constructs a BreakerDoubleSolenoid.
     * 
     * @param moduleType   Module type used (CTRE or REV).
     * @param channel      Solenoid channel.
     * @param pcmID        ID for PCM.
     * @param defaultState Default solenoid state.
     */
    public BreakerSolenoid(PneumaticsModuleType moduleType, int channel, int pcmID, boolean defaultState) {
        solenoid = new Solenoid(pcmID, moduleType, channel);
        setDefaultState(defaultState);
        toDefaultState();
    }

    /**
     * Sets solenoid's default state to the given value.
     * 
     * @param value True = extended, false = not extended.
     */
    public void setDefaultState(boolean value) {
        defaultState = value;
    }

    /** Resets solenoid to its default state. */
    public void toDefaultState() {
        if (solenoid.get() != defaultState) {
            solenoid.set(defaultState);
        }
    }

    /**
     * Sets solenoid to given state.
     * 
     * @param value True = extended, false = not extended.
     */
    public void set(boolean value) {
        if (solenoid.get() != value) {
            solenoid.set(value);
        }
    }

    /** @return State of solenoid (true = extended, false = not extended). */
    public boolean getState() {
        return solenoid.get();
    }

    /** Makes solenoid switch between extended and not extended. */
    public void toggleState() {
        solenoid.toggle();
    }
}
