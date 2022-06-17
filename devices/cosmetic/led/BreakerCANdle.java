// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.led;

import java.util.HashMap;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.util.BreakerCTREUtil;
import frc.robot.BreakerLib.util.BreakerTriplet;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerChannel;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.powermanagement.DevicePowerMode;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.selftest.SelfTest;

/** LED controller */
public class BreakerCANdle extends SubsystemBase implements BreakerGenericDevice {

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
    private DeviceHealth health = DeviceHealth.NOMINAL;
    private String faults;
    private String diviceName = " CANdle_LED_Controller ";
    private int canID;

    public BreakerCANdle(int canID, int numberOfLEDs, BreakerCANdleConfig config) {
        candle = new CANdle(canID);
        candle.configAllSettings(config.getConfig());
        candle.setLEDs(255, 255, 255);
        enabledStatus = new RainbowAnimation(1, 0.5, numberOfLEDs);
        errorStatus = new StrobeAnimation(255, 0, 0, 0, 0.5, numberOfLEDs);
        this.canID = canID;
        SelfTest.autoRegesterDevice(this);
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

    @Override
    public void runSelfTest() {
        faults = null;
        CANdleFaults faultsC = new CANdleFaults();
        candle.getFaults(faultsC);
        if (faultsC.hasAnyFault() || SelfTest.checkIsMissingCanID(canID)) {
            BreakerTriplet<DeviceHealth, String, Boolean> faultData = BreakerCTREUtil.getCANdelHealthFaultsAndConnectionStatus(faultsC, canID);
            faults = faultData.getMiddle();
            health = faultData.getLeft();
        } else {
            health = DeviceHealth.NOMINAL;
        }
        
    }

    @Override
    public DeviceHealth getHealth() {
        return health;
    }

    @Override
    public String getFaults() {
        return faults;
    }

    @Override
    public String getDeviceName() {
        return diviceName;
    }

    @Override
    public boolean hasFault() {
        return health != DeviceHealth.NOMINAL;
    }

    @Override
    public void setDeviceName(String newName) {
        diviceName = newName;
    }

    @Override
    public boolean isUnderAutomaticControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public DevicePowerMode getPowerMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void returnToAutomaticPowerManagement() {
        // TODO Auto-generated method stub
        
    }
}
