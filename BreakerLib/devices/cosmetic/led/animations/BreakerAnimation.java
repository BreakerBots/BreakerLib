// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led.animations;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public interface BreakerAnimation {
    public abstract Color8Bit getColor8BitAtIndex(double time, int index);
    public abstract Color getColorAtIndex(double time, int index);
    public abstract AddressableLEDBuffer getBuffer(double time);
    public abstract int getLength();

}
