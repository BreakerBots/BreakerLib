// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.BreakerLib.devices.cosmetic.led.animations.BreakerAnimation;
import frc.robot.BreakerLib.util.BreakerRoboRIO.RobotMode;

/** Driver for all LED controllers in BreakerLib. */
public interface BreakerGenericLEDDriver {
    public static final Color BREAKER_RED = new Color(new Color8Bit(196, 30, 58));
    public static final Color BREAKER_GOLD = new Color(new Color8Bit(253, 181, 21));

    public abstract void setAllLEDs(int r, int g, int b);

    public default void setAllLEDs(Color ledColor) {
        Color8Bit col = new Color8Bit(ledColor);
        setAllLEDs(col.red, col.green, col.blue);
    }
    public default void setAllLEDs(Color8Bit ledColor) {
        setAllLEDs(ledColor.red, ledColor.green, ledColor.blue);
    }


    public abstract void setLEDsInRange(int r, int g, int b, int startIndex, int endIndex);
    public default void setLEDsInRange(Color ledColor, int startIndex, int endIndex) {
        Color8Bit col = new Color8Bit(ledColor);
        setAllLEDs(col.red, col.green, col.blue);
    }
    public default void setLEDsInRange(Color8Bit ledColor, int startIndex, int endIndex) {
        setAllLEDs(ledColor.red, ledColor.green, ledColor.blue);
    }


    public abstract void setLED(int r, int g, int b, int index);
    public default void setLED(Color ledColor, int index) {
        Color8Bit col = new Color8Bit(ledColor);
        setLED(col.red, col.green, col.blue, index);
    }
    public default void setLED(Color8Bit ledColor, int index) {
        setLED(ledColor.red, ledColor.green, ledColor.blue, index);
    }

    public abstract void setAnimation(BreakerAnimation state);
    public abstract void setAnimation(int startIndex, BreakerAnimation state);
    public abstract void setAnimation(int startIndex, int endIndex, BreakerAnimation state);

    public abstract void setRobotModeDefaultState(RobotMode mode, BreakerAnimation state);

    /** clears all active effects to the set mode default */
    public abstract void clearToModeDefault();

    public abstract void clearActiveAnimation();

    public abstract void setOn();

    public abstract void setOff();

    public abstract boolean isOn();

    public abstract boolean isInModeDefault();


}
