// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import java.time.zone.ZoneOffsetTransitionRule.TimeDefinition;
import java.util.HashMap;
import java.util.Map;

import org.opencv.features2d.FastFeatureDetector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.cosmetic.led.animations.BreakerAnimation;
import frc.robot.BreakerLib.util.BreakerRoboRIO;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotMode;

/** Add your docs here. */
public class BreakerPWMLED extends SubsystemBase implements BreakerGenericLEDDriver {
    private AddressableLED led;
    private AddressableLEDBuffer buff;
    private int stripLength, animationStartIndex, animationEndIndex;
    private BreakerAnimation curAnimation;
    private boolean isOn = false, isActiveAnimation = false, isDefault = true;
    private double curAnimationStartTime;
    private Map<RobotMode, BreakerAnimation> defaultRobotModeAnims = new HashMap<>();
    public BreakerPWMLED(int portPWM, int stripLength) {
        led = new AddressableLED(portPWM);
        led.setLength(stripLength);
        buff = new AddressableLEDBuffer(stripLength);
        this.stripLength = stripLength;
    }

    public void setBitTiming(int lowTime0NanoSeconds, int highTime0NanoSeconds, int lowTime1NanoSeconds, int highTime1NanoSeconds) {
        led.setBitTiming(lowTime0NanoSeconds, highTime0NanoSeconds, lowTime1NanoSeconds, highTime1NanoSeconds);
    }
    
    @Override
    public void setAllLEDs(int r, int g, int b) {
        Color8Bit col = new Color8Bit(r, g, b);
        for (int i = 0; i < stripLength; i++) {
            buff.setLED(i, col);
        }
        led.setData(buff);
        isDefault = false;
        isActiveAnimation = false;
    }
    @Override
    public void setLEDsInRange(int r, int g, int b, int startIndex, int endIndex) {
        startIndex = MathUtil.clamp(startIndex, 0, stripLength - 1);
        startIndex = MathUtil.clamp(endIndex, startIndex, stripLength - 1);
        for (int i = startIndex; i < endIndex; i++) {
            buff.setRGB(i, r, g, b);
        }
        led.setData(buff);
        isDefault = false;
        isActiveAnimation = false;
    }

    @Override
    public void setLED(int r, int g, int b, int index) {
        index = MathUtil.clamp(index, 0, stripLength - 1);
        buff.setRGB(index, r, g, b);
        isDefault = false;
        isActiveAnimation = false;
    }
    
    @Override
    public void setAnimation(BreakerAnimation state) {
        curAnimation = state;
        animationStartIndex = 0;
        animationEndIndex = state.getLength();
        isActiveAnimation = true;
        isDefault = false;
        curAnimationStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setAnimation(int startIndex, BreakerAnimation state) {
        curAnimation = state;
        animationStartIndex = startIndex;
        animationEndIndex = state.getLength();
        isActiveAnimation = true;
        isDefault = false;
        curAnimationStartTime = Timer.getFPGATimestamp();
        
    }
    @Override
    public void setAnimation(int startIndex, int endIndex, BreakerAnimation state) {
        curAnimation = state;
        animationStartIndex = startIndex;
        animationEndIndex = endIndex;
        isActiveAnimation = true;
        isDefault = false;
        curAnimationStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setRobotModeDefaultState(RobotMode mode, BreakerAnimation state) {
        if (defaultRobotModeAnims.containsKey(mode)) {
            defaultRobotModeAnims.replace(mode, state);
        } else {
            defaultRobotModeAnims.put(mode, state);
        }
    }

    @Override
    public void clearToModeDefault() {
        isActiveAnimation = false;
        isDefault = true;
    }

    @Override
    public void setOn() {
        led.start();
    }

    @Override
    public void setOff() {
       isOn = false;
       setAllLEDs(0, 0, 0);
       led.stop();
    }

    @Override
    public boolean isOn() {
        return isOn;
    }

    @Override
    public void clearActiveAnimation() {
        isActiveAnimation = false;
    }

    @Override
    public boolean isInModeDefault() {
        return isDefault;
    }
 
    @Override
    public void periodic() {
        if (isOn) {
            if (isActiveAnimation && !isDefault) {
                int interalIndex = 0;
                for (int i = animationStartIndex; i <= animationEndIndex; i++) {
                    buff.setLED(i, curAnimation.getColorAtIndex(Timer.getFPGATimestamp() - curAnimationStartTime, interalIndex++));
                }
            } else if (isDefault) {
                for (int i = 0; i < stripLength; i++) {
                    buff = defaultRobotModeAnims.get(BreakerRoboRIO.getCurrentRobotMode()).getBuffer(Timer.getFPGATimestamp() - curAnimationStartTime);
                }
            }

        }
    }
    
}
