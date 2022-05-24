// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;

public class BreakerColorSensor implements BreakerGenericDevice {

  private DeviceHealth currentHealth = DeviceHealth.NOMINAL;
  private ColorSensorV3 colorSensor;
  private String faults = null;
  private String deviceName = "Color_Sensor_V3";

  public BreakerColorSensor(Port i2cPort) {
    colorSensor = new ColorSensorV3(i2cPort);
  }

  public Color getColor() {
    return colorSensor.getColor();
  }

  public boolean compareColors(Color comparisionColor) {
    return (comparisionColor == colorSensor.getColor());
  }


  public int[] getRawColorsADC() {
    int[] colorVals = new int[4];
    colorVals[0] = colorSensor.getRawColor().red;
    colorVals[1] = colorSensor.getRawColor().green;
    colorVals[2] = colorSensor.getRawColor().blue;
    colorVals[3] = colorSensor.getRawColor().ir;
    return colorVals;
  }

  /** returns the sensors proximity to its sensing target with 2047 being closest and 0 being furthest */
  public int getProximity() {
    return colorSensor.getProximity();
  }

  @Override
  public void runSelfTest() {
    faults = null;
    if (!colorSensor.isConnected()) {
      currentHealth = DeviceHealth.INOPERABLE;
      faults = " COLOR_SENSOR_NOT_CONNECTED ";
    } else {
      currentHealth = DeviceHealth.NOMINAL;
      faults = null;
    }
  }

  @Override
  public DeviceHealth getHealth() {
    return currentHealth;
  }

  @Override
  public String getFaults() {
    return faults;
  }

  @Override
  public String getDeviceName() {
    return deviceName;
  }

  @Override
  public boolean hasFault() {
    return false;
  }

  @Override
  public void setDeviceName(String newName) {
    deviceName = newName;
    
  }

  
}
