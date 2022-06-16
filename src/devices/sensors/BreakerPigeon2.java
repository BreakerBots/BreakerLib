// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.BreakerLib.devices.sensors;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.position.geometry.BreakerRotation3d;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerChannel;
import frc.robot.BreakerLib.util.powermanagement.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.powermanagement.DevicePowerMode;
import frc.robot.BreakerLib.util.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.selftest.SelfTest;

/* Good version of the CTRE Pigeon 2 class BAYBEEE! */
public class BreakerPigeon2 implements BreakerGenericDevice {
  private WPI_Pigeon2 pigeon;
  private double imuInvert;
  private DeviceHealth currentHealth = DeviceHealth.NOMINAL;
  private String faults = null;
  private String deviceName = "Pigeon2_IMU";

  /** Creates a new PigeonIMU object. */
  public BreakerPigeon2(int deviceID) {
    pigeon = new WPI_Pigeon2(deviceID);
    SelfTest.autoRegesterDevice(this);
  }

  /** Returns pitch angle within +- 180 degrees */
  public double getPitchDegrees() {
    return BreakerMath.angleModulus(pigeon.getPitch());
  }

  /** Returns yaw angle within +- 180 degrees */
  public double getYawDegrees() {
    return BreakerMath.angleModulus(pigeon.getYaw());
  }

  /** Returns roll angle within +- 180 degrees */
  public double getRollDegrees() {
    return BreakerMath.angleModulus(pigeon.getRoll());
  }

  public Rotation2d getPitch() {
    return Rotation2d.fromDegrees(getPitchDegrees());
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(getYawDegrees());
  }

  public Rotation2d getRoll() {
    return Rotation2d.fromDegrees(getRollDegrees());
  }

  public BreakerRotation3d getRotation3d() {
    return new BreakerRotation3d(getPitch(), getYaw(), getRoll());
  }

  /**
   * Returns raw yaw, pitch, and roll angles in an array.
   * <p>
   * yaw = 0, pitch = 1, roll = 2.
   */
  public double[] getRawAngles() {
    double[] RawYPR = new double[3];
    pigeon.getYawPitchRoll(RawYPR);
    return RawYPR;
  }

  /** Resets yaw to 0 degrees */
  public void reset() {
    pigeon.setYaw(0);
  }

  /** Sets yaw to given angle. */
  public void set(double angle) {
    pigeon.setYaw(angle);
  }

  /** Returns accelerometer value based on given index */
  public double getGyroRates(int arrayElement) {
    double[] rawRates = new double[3];
    pigeon.getRawGyro(rawRates);
    return rawRates[arrayElement];
  }

  /** Accelerometer pitch. */
  public double getPitchRate() {
    return getGyroRates(0);
  }

  /** Accelerometer yaw. */
  public double getYawRate() {
    return getGyroRates(1);
  }

  /** Accelerometer roll. */
  public double getRollRate() {
    return getGyroRates(2);
  }

  /**
   * Returns array of raw accelerometer values.
   * <p>
   * x = 0, y = 1, z = 2.
   */
  public short[] getRawAccelerometerVals() {
    short[] accelVals = new short[3];
    pigeon.getBiasedAccelerometer(accelVals);
    return accelVals;
  }

  /** Accelerometer x-value in inches */
  public double getRawIns2AccelX() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals()[0], 14) * 0.02);
  }

  /** Accelerometer y-value in inches */
  public double getRawIns2AccelY() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals()[1], 14) * 0.02);
  }

  /** Accelerometer z-value in inches */
  public double getRawIns2AccelZ() {
    return (BreakerMath.fixedToFloat(getRawAccelerometerVals()[2], 14) * 0.02);
  }

  /** How long the Pigeon has been running for, in seconds. Maxes out at 255 sec.*/
  public int getPigeonUpTime() {
    return pigeon.getUpTime();
  }

  public BreakerRotation3d getRawRotation3d() {
    return new BreakerRotation3d(Rotation2d.fromDegrees(getRawAngles()[1]), Rotation2d.fromDegrees(getRawAngles()[0]),
        Rotation2d.fromDegrees(getRawAngles()[2]));
  }

  @Override
  public void runSelfTest() {
    faults = null;
    Pigeon2_Faults curFaults = new Pigeon2_Faults();
    pigeon.getFaults(curFaults);
    
    if (curFaults.HardwareFault) {
      currentHealth = DeviceHealth.INOPERABLE;
      faults += " HARDWARE_FAULT ";
    }
    if (curFaults.MagnetometerFault) {
      currentHealth = DeviceHealth.INOPERABLE;
      faults += " MAG_FAULT ";
    }
    if (curFaults.GyroFault) {
      currentHealth = DeviceHealth.INOPERABLE;
      faults += "  GYRO_FAULT ";
    }
    if (curFaults.AccelFault) {
      currentHealth = DeviceHealth.INOPERABLE;
      faults += "  ACCEL_FAULT ";
    }
    if (curFaults.UnderVoltage) {
      currentHealth = (currentHealth != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : currentHealth;
      faults += " UNDER_6.5V ";
    }
    if (!curFaults.HardwareFault && !curFaults.MagnetometerFault && !curFaults.GyroFault
        && !curFaults.AccelFault && !curFaults.UnderVoltage) {
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
    return currentHealth != DeviceHealth.NOMINAL;
  }

  @Override
  public void setDeviceName(String newName) {
    deviceName = newName;
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
