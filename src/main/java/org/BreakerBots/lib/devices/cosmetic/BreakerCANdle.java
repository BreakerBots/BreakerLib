// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** LED controller */
public class BreakerCANdle extends SubsystemBase {

    public enum BreakerCANdleLedMode {
        COLOR_SWITCH,
        ANIMATION,
        STATIC,
        ERROR,
        ENABLED,
        DISSABLED,
        OFF
    }

    private CANdle candle;
    private RainbowAnimation enabledStatus;
    private StrobeAnimation errorStatus;
    private int colorSwitch = 0;
    private int timer = 0;
    private double switchTimeSec;
    private Color[] switchColors;
    private BreakerCANdleLedMode mode = BreakerCANdleLedMode.OFF;

    public BreakerCANdle(int canID, int numberOfLEDs, BreakerCANdleConfig config) {
        candle = new CANdle(canID);
        candle.configAllSettings(config.getConfig());
        candle.setLEDs(255, 255, 255);
        enabledStatus = new RainbowAnimation(1, 0.5, numberOfLEDs);
        errorStatus = new StrobeAnimation(255, 0, 0, 0, 0.5, numberOfLEDs);
    }

    public void setLedAnimation(Animation animation) {
        candle.animate(animation);
        mode = BreakerCANdleLedMode.ANIMATION;
    }

    public void setRobotEnabledStatusLED() {
        mode = BreakerCANdleLedMode.ENABLED;
    }

    public void setLedOff() {
        mode = BreakerCANdleLedMode.OFF;
    }

    public void runErrorStatusLED() {
        mode = BreakerCANdleLedMode.ERROR;
    }

    public void setStaticLED(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
        mode = BreakerCANdleLedMode.STATIC;
    }

    public void setStaticLED(Color ledColor) {
        candle.setLEDs(colorToRGB(ledColor)[0], colorToRGB(ledColor)[1], colorToRGB(ledColor)[2]);
        mode = BreakerCANdleLedMode.STATIC;
    }

    private void setLED(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }

    private int[] colorToRGB(Color color) {
        return new int[] { (int) (color.red * 255), (int) (color.blue * 255), (int) (color.green * 255) };
    }

    private void setLED(Color ledColor) {
        candle.setLEDs(colorToRGB(ledColor)[0], colorToRGB(ledColor)[1], colorToRGB(ledColor)[2]);
    }

    public void setLedColorSwitch(double switchTimeSec, Color... switchColors) {
        this.switchTimeSec = switchTimeSec;
        this.switchColors = switchColors;
        colorSwitch = 0;
        mode = BreakerCANdleLedMode.COLOR_SWITCH;
    }

    private void runColorSwitch(double switchTimeSec, Color... colors) {
        if (timer % (switchTimeSec * 50) == 0) {
            colorSwitch = (colorSwitch > colors.length) ? 0 : colorSwitch;
            setLED(colors[colorSwitch++]);
        }
    }

    private void runLED() {
        switch (mode) {
            default:
            case OFF:
                setLED(0, 0, 0);
                break;
            case COLOR_SWITCH:
                runColorSwitch(switchTimeSec, switchColors);
                break;
            case ERROR:
                candle.animate(errorStatus);
                break;
            case ENABLED:
                candle.animate(enabledStatus);
                break;
            case DISSABLED:
                setLED(Color.kOrange);
                break;
            case ANIMATION:
            case STATIC:
                break;
        }
    }

    @Override
    public void periodic() {
        timer++;
        runLED();
    }
}
