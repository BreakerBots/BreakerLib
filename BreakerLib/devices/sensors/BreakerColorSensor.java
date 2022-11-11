// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.BreakerLib.devices.BreakerGenericDeviceBase;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** REV Color Sensor V3 implementing the Breaker device interface. */
public class BreakerColorSensor extends BreakerGenericDeviceBase {

  private ColorSensorV3 colorSensor;
 

  /**
   * Create a new BreakerColorSensor.
   * 
   * @param i2cPort I2C port for the color sensor.
   */
  public BreakerColorSensor(Port i2cPort) {
    colorSensor = new ColorSensorV3(i2cPort);
    deviceName = "Color_Sensor_V3";
  }

  /** Current color detected by the sensor. */
  public Color getColor() {
    return colorSensor.getColor();
  }

  /** Compare target color to detected color. */
  public boolean compareColors(Color comparisionColor) {
    return (comparisionColor == colorSensor.getColor());
  }

  /** Delivers RGB values plus IR value. */
  public int[] getRawColorsADC() {
    int[] colorVals = new int[4];
    colorVals[0] = colorSensor.getRawColor().red;
    colorVals[1] = colorSensor.getRawColor().green;
    colorVals[2] = colorSensor.getRawColor().blue;
    colorVals[3] = colorSensor.getRawColor().ir;
    return colorVals;
  }

  /**
   * Returns the sensor's proximity to its sensing target with 2047 being closest
   * and 0 being furthest.
   */
  public int getProximity() {
    return colorSensor.getProximity();
  }

  @Override
  public void runSelfTest() {
    faultStr = null;
    if (!colorSensor.isConnected()) {
      health = DeviceHealth.INOPERABLE;
      faultStr = " COLOR_SENSOR_NOT_CONNECTED ";
    } else {
      health = DeviceHealth.NOMINAL;
      faultStr = null;
    }
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
