// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Util class for CTRE motors. */
public class BreakerCTREMotorUtil {

  /**
   * 
   * @param isEnabled True for brake mode, false for coast mode.
   * @param motors Talon FX motors.
   */
  public static void setBrakeMode(boolean isEnabled, WPI_TalonFX... motors) {
    for (WPI_TalonFX motor : motors) {
      motor.setNeutralMode((isEnabled ? NeutralMode.Brake : NeutralMode.Coast));
    }
  }

  public static WPI_TalonFX[] createMotorArray(WPI_TalonFX... controllers) {
    return controllers;
  }

  public static void checkError(ErrorCode error, String Message) {
    if (error != ErrorCode.OK) {
      BreakerLog.logError(error + " - " + Message);
    }
  }

  /**
   * @param motorFaults Faults from a CTRE Motor Controller
   * @return All motor fault messages separated by spaces in a string.
   */
  public static String getMotorFaultsAsString(Faults motorFaults) {
    StringBuilder work = new StringBuilder();
    if (motorFaults.hasAnyFault()) {
      int field = motorFaults.toBitfield(); // Field for examining errors
      int fieldMask = 1; // Need explanation

      for (int fieldPlace = 0; fieldPlace <= 13; fieldPlace++) {
        if ((field & fieldMask) != 0) { // Checks for 1s in bitfield that signifies error
          switch (fieldPlace) { // Finds type of error based on bit location
            case 0:
              work.append(" device_under_6.5v ");
              break;
            case 1:
            case 2:
            case 3:
            case 4:
              work.append(" device_limit_switch_hit ");
              break;
            case 5:
              work.append(" hardware_failure ");
              break;
            case 6:
            case 9:
              work.append(" device_activated_or_reset_while_robot_on ");
              break;
            case 7:
              work.append(" sensor_overflow ");
              break;
            case 8:
              work.append(" sensor_out_of_phase ");
              break;
            case 10:
              work.append(" remote_sensor_not_detected ");
              break;
            case 11:
              work.append(" API_or_firmware_error ");
              break;
            case 12:
              work.append(" supply_voltage_above_rated_max ");
              break;
            case 13:
              work.append(" unstable_supply_voltage ");
              break;
          }
        }
        fieldMask <<= 1; // Scrolls to next bit.
      }
    } else {
      work.append(" none ");
    }
    return work.toString();
  }
}
