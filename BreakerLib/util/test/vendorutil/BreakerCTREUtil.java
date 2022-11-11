// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.vendorutil;

import java.util.HashMap;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdleFaults;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoderFaults;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Util class for CTRE devices */
public class BreakerCTREUtil {

  /**
   * Sets brake mode for given TalonFX motors.
   * 
   * @param isEnabled True for brake mode, false for coast mode.
   * @param motors    Talon FX motors.
   */
  public static void setBrakeMode(boolean isEnabled, TalonFX... motors) {
    for (TalonFX motor : motors) {
      motor.setNeutralMode((isEnabled ? NeutralMode.Brake : NeutralMode.Coast));
    }
  }

  /**
   * Logs an error to BreakerLog if designated error is discovered.
   * 
   * @param error   CTRE error code to detect.
   * @param message Message to log when the error is detected.
   */
  public static void checkError(ErrorCode error, String message) {
    if (error != ErrorCode.OK) {
      BreakerLog.logError(error + " - " + message);
    }
  }

  /**
   * Returns motor faults as a String.
   * 
   * @param motorFaults Faults from a CTRE motor controller.
   * @return All motor fault messages separated by spaces in a string.
   */
  public static String getMotorFaultsAsString(Faults motorFaults) {
    HashMap<Integer, String> map = new HashMap<Integer, String>();
    map.put(0, " device_under_6.5v ");
    map.put(1, " device_limit_switch_hit ");
    map.put(2, " device_limit_switch_hit ");
    map.put(3, " device_limit_switch_hit ");
    map.put(4, " device_limit_switch_hit ");
    map.put(5, " hardware_failure ");
    map.put(6, " device_activated_or_reset_while_robot_on ");
    map.put(9, " device_activated_or_reset_while_robot_on ");
    map.put(7, " sensor_overflow ");
    map.put(8, " sensor_out_of_phase ");
    map.put(10, " remote_sensor_not_detected ");
    map.put(11, " API_or_firmware_error ");
    map.put(12, " supply_voltage_above_rated_max ");
    map.put(13, " unstable_supply_voltage ");
    return getDeviceFaultsAsString(motorFaults.toBitfield(), map);
  }

  /**
   * Get CTRE motor controller faults and device health.
   * 
   * @param motorFaults Motor controller faults.
   * @return Motor controller device health and error type (if any).
   */
  public static Pair<DeviceHealth, String> getMotorHealthAndFaults(Faults motorFaults) {
    HashMap<Integer, Pair<DeviceHealth, String>> map = new HashMap<>();
    map.put(0, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_under_6.5v "));
    map.put(1, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    map.put(2, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    map.put(3, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    map.put(4, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_limit_switch_hit "));
    map.put(5, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " hardware_failure "));
    map.put(6, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_activated_or_reset_while_robot_on "));
    map.put(9, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " device_activated_or_reset_while_robot_on "));
    map.put(7, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " sensor_overflow "));
    map.put(8, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " sensor_out_of_phase "));
    map.put(10, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " remote_sensor_not_detected "));
    map.put(11, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " API_or_firmware_error "));
    map.put(12, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " supply_voltage_above_rated_max "));
    map.put(13, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " unstable_supply_voltage "));
    return BreakerVendorUtil.getDeviceHealthAndFaults(motorFaults.toBitfield(), map);
  }

  /**
   * Get CANCoder faults and device health.
   * 
   * @param encoderFaults CANCoder faults.
   * @return CANCoder device health and error type (if any).
   */
  public static Pair<DeviceHealth, String> getCANCoderHealthAndFaults(CANCoderFaults encoderFaults) {
    HashMap<Integer, Pair<DeviceHealth, String>> map = new HashMap<>();
    map.put(0, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " hardware_failure "));
    map.put(4, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " magnet_too_weak "));
    return BreakerVendorUtil.getDeviceHealthAndFaults(encoderFaults.toBitfield(), map);
  }

  /** Get motor faults, fault name, and connection status. */
  // public static BreakerTriplet<DeviceHealth, String, Boolean>
  // getMotorHealthFaultsAndConnectionStatus(
  // Faults motorFaults, int deviceID) {
  // HashMap<Integer, Pair<DeviceHealth, String>> map = new HashMap<>();
  // map.put(0, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // device_under_6.5v "));
  // map.put(1, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // device_limit_switch_hit "));
  // map.put(2, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // device_limit_switch_hit "));
  // map.put(3, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // device_limit_switch_hit "));
  // map.put(4, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // device_limit_switch_hit "));
  // map.put(5, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, "
  // hardware_failure "));
  // map.put(6, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // device_activated_or_reset_while_robot_on "));
  // map.put(9, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // device_activated_or_reset_while_robot_on "));
  // map.put(7, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // sensor_overflow "));
  // map.put(8, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // sensor_out_of_phase "));
  // map.put(10, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // remote_sensor_not_detected "));
  // map.put(11, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // API_or_firmware_error "));
  // map.put(12, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // supply_voltage_above_rated_max "));
  // map.put(13, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, "
  // unstable_supply_voltage "));
  // return GetDeviceHealthFaultsAndConnectionStatus(motorFaults.toBitfield(),
  // deviceID, map);
  // }

  /**
   * Get CANdle faults and device health.
   * 
   * @param faults CANCoder faults.
   * @return CANCoder device health and error type (if any).
   */
  public static Pair<DeviceHealth, String> getCANdleHealthAndFaults(CANdleFaults faults) {
    HashMap<Integer, Pair<DeviceHealth, String>> map = new HashMap<>();
    map.put(0, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " short_circut "));
    map.put(1, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " thermal_fault "));
    map.put(2, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " software_fuse "));
    map.put(8, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " API_error "));
    map.put(9, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " hardware_failure "));
    return BreakerVendorUtil.getDeviceHealthAndFaults(faults.toBitfield(), map);
  }

  // /** Gets CANdle faults, name, and connection status. */
  // public static BreakerTriplet<DeviceHealth, String, Boolean>
  // getCANdelHealthFaultsAndConnectionStatus(
  // CANdleFaults faults, int deviceID) {
  // HashMap<Integer, Pair<DeviceHealth, String>> map = new HashMap<>();
  // map.put(0, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, "
  // short_circut "));
  // map.put(1, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " thermal_fault
  // "));
  // map.put(2, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, "
  // software_fuse "));
  // map.put(8, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " API_error
  // "));
  // map.put(9, new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, "
  // hardware_failure "));
  // return GetDeviceHealthFaultsAndConnectionStatus(faults.toBitfield(),
  // deviceID, map);
  // }

  /**
   * Gets CANdle faults as a String. 
   * 
   * @param faults CANdle faults.
   * @return All faults found as string. If no faults are found, "none" is returned.
  */
  public static String getCANdleFaultsAsString(CANdleFaults faults) {
    HashMap<Integer, String> map = new HashMap<Integer, String>();
    map.put(0, " short_circut ");
    map.put(1, " thermal_fault ");
    map.put(2, " software_fuse ");
    map.put(8, " API_error ");
    map.put(9, " hardware_failure ");
    return getDeviceFaultsAsString(faults.toBitfield(), map);
  }

  /**
   * Gets CTRE device faults as string 
   * 
   * @param Device faults as bitfield
   * @return All faults found as string. If no faults are found, "none" is returned.
  */
  public static String getDeviceFaultsAsString(long faultBitField,
      HashMap<Integer, String> fieldPlacesAndFaultMessages) {
    StringBuilder work = new StringBuilder();
    if (faultBitField != 0) {
      long fieldMask = 1; // masks all but selected bit
      for (int fieldPlace = 0; fieldPlace < fieldPlacesAndFaultMessages.size(); fieldPlace++) {
        if (((faultBitField & fieldMask) != 0) && fieldPlacesAndFaultMessages.containsKey(fieldPlace)) { // Checks for
                                                                                                         // 1s in
                                                                                                         // bitfield
                                                                                                         // that
                                                                                                         // signifies
                                                                                                         // error
          work.append(fieldPlacesAndFaultMessages.get(fieldPlace));
        }
        fieldMask <<= 1; // Scrolls to next bit.
      }
    } else {
      work.append(" none ");
    }
    return work.toString();
  }

  /** Get device health, faults, and connection status. */
  // public static BreakerTriplet<DeviceHealth, String, Boolean>
  // GetDeviceHealthFaultsAndConnectionStatus(
  // long faultBitField, int deviceID,
  // HashMap<Integer, Pair<DeviceHealth, String>>
  // fieldPlacesHealthEffectsAndFaultMessages) {
  // Pair<DeviceHealth, String> healthAndMsgs =
  // BreakerVendorUtil.getDeviceHealthAndFaults(faultBitField,
  // fieldPlacesHealthEffectsAndFaultMessages);
  // boolean isMissing = SelfTest.checkIsMissingCanID(deviceID);
  // String messages = isMissing ? healthAndMsgs.getSecond() + "
  // device_not_found_on_bus " : healthAndMsgs.getSecond();
  // return new BreakerTriplet<DeviceHealth, String, Boolean>(
  // isMissing ? DeviceHealth.INOPERABLE : healthAndMsgs.getFirst(), messages,
  // isMissing);
  // }
}
