// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*; // Just learned how to statically import enum values WOW!

/** DoubleSolenoid with support for default state. */
public class BreakerDoubleSolenoid {
    private DoubleSolenoid solenoid;
    private Value defaultState = kOff;

    /** Constructs a BreakerDoubleSolenoid.
     * 
     * @param moduleType Module type used (CTRE or REV).
     * @param fwdChannel Forward air channel.
     * @param revChannel Reverse air channel.
     */
    public BreakerDoubleSolenoid(PneumaticsModuleType moduleType, int fwdChannel, int revChannel) {
        solenoid = new DoubleSolenoid(moduleType, fwdChannel, revChannel);
        toDefaultState();
    }

     /** Constructs a BreakerDoubleSolenoid.
     * 
     * @param moduleType Module type used (CTRE or REV).
     * @param fwdChannel Forward air channel.
     * @param revChannel Reverse air channel.
     * @param pcmID ID for PCM.
     */
    public BreakerDoubleSolenoid(PneumaticsModuleType moduleType, int fwdChannel, int revChannel, int pcmID) {
        solenoid = new DoubleSolenoid(pcmID, moduleType, fwdChannel, revChannel);
        toDefaultState();
    }

     /** Constructs a BreakerDoubleSolenoid.
     * 
     * @param moduleType Module type used (CTRE or REV).
     * @param fwdChannel Forward air channel.
     * @param revChannel Reverse air channel.
     * @param defaultState Default solenoid state.
     */
    public BreakerDoubleSolenoid(PneumaticsModuleType moduleType, int fwdChannel, int revChannel, Value defaultState) {
        solenoid = new DoubleSolenoid(moduleType, fwdChannel, revChannel);
        setDefaultState(defaultState);
        toDefaultState();
    }

     /** Constructs a BreakerDoubleSolenoid.
     * 
     * @param moduleType Module type used (CTRE or REV).
     * @param fwdChannel Forward air channel.
     * @param revChannel Reverse air channel.
     * @param pcmID ID for PCM.
     * @param defaultState Default solenoid state.
     */
    public BreakerDoubleSolenoid(PneumaticsModuleType moduleType, int fwdChannel, int revChannel, int pcmID, Value defaultState) {
        solenoid = new DoubleSolenoid(pcmID, moduleType, fwdChannel, revChannel);
        setDefaultState(defaultState);
        toDefaultState();
    }

    /** Sets solenoid's default state to the given value. */
    public void setDefaultState(Value value) {
        defaultState = value;
    }

    /** Resets solenoid to its default state. */
    public void toDefaultState() {
        if (solenoid.get() != defaultState) {
            solenoid.set(defaultState);
        }
    }

    /** Sets solenoid to given state.
     * 
     * @param value kForward for extension, kReverse for retraction, kOff to deactivate.
     */
    public void set(Value value) {
        if (solenoid.get() != value) {
            solenoid.set(value);
        }
    }

    public Value getState() {
        return solenoid.get();
    }

    /** Makes solenoid switch between forward and reverse. */
    public void toggleState() {
        solenoid.toggle();
    }
}
