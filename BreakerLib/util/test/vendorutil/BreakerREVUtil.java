// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.vendorutil;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/** Util class for REV devices. */
public class BreakerREVUtil {

  /**
   * Sets brake mode for given SparkMax motors.
   * 
   * @param isEnabled True for brake mode, false for coast mode.
   * @param motors    SparkMax motors.
   */
  public static void setBrakeMode(boolean isEnabled, CANSparkMax... motors) {
    for (CANSparkMax motor : motors) {
      motor.setIdleMode((isEnabled ? IdleMode.kBrake : IdleMode.kCoast));
    }
  }

  /**
   * Returns SparkMax motor faults as a String.
   * 
   * @param motorFaults Faults from a SparkMax motor controller.
   * @return Device health status and string of discovered faults.
   */
  public static Pair<DeviceHealth, String> getSparkMaxHealthAndFaults(short motorFaults) {
    HashMap<Integer, Pair<DeviceHealth, String>> map = new HashMap<>();
    map.put(FaultID.kBrownout.value, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " brownout "));
    map.put(FaultID.kCANRX.value, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " CAN_Receive_error "));
    map.put(FaultID.kCANTX.value, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " CAN_transmit_error "));
    map.put(FaultID.kDRVFault.value, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " DRV_fault "));
    map.put(FaultID.kEEPROMCRC.value, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " EEPROM_fault "));
    map.put(FaultID.kMotorFault.value,
        new Pair<DeviceHealth, String>(DeviceHealth.INOPERABLE, " general_motor_fault "));
    map.put(FaultID.kOtherFault.value, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " general_fault "));
    map.put(FaultID.kOvercurrent.value, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " overcurrent "));
    map.put(FaultID.kSensorFault.value, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " sensor_fault "));
    map.put(FaultID.kSoftLimitRev.value, new Pair<DeviceHealth, String>(DeviceHealth.FAULT, " motor_stall "));
    return BreakerVendorUtil.getDeviceHealthAndFaults(motorFaults, map);
  }
}