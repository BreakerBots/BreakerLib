// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led.animations;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class BreakerStaticAnimation implements BreakerAnimation {
    private Color staticColor;
    private AddressableLEDBuffer buff;
    private int length;
    public BreakerStaticAnimation(Color8Bit staticColor, int length) {
        this.staticColor = new Color(staticColor);
        buff = new AddressableLEDBuffer(length);
        for (int i = 0; i < length; i++) {
            buff.setLED(i, staticColor);
        }
        this.length = length;
    }

    public BreakerStaticAnimation(Color staticColor, int length) {
        this.staticColor = staticColor;
        buff = new AddressableLEDBuffer(length);
        for (int i = 0; i < length; i++) {
            buff.setLED(i, staticColor);
        }
        this.length = length;
    }

    @Override
    public Color8Bit getColor8BitAtIndex(double time, int index) {
        return new Color8Bit(staticColor);
    }

    @Override
    public Color getColorAtIndex(double time, int index) {
        return staticColor;
    }

    @Override
    public AddressableLEDBuffer getBuffer(double time) {
        return buff;
    }

    @Override
    public int getLength() {
        return length;
    }
    
}
