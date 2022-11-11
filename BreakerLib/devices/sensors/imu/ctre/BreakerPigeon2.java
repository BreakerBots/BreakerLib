// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.BreakerLib.devices.sensors.imu.ctre;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import frc.robot.BreakerLib.devices.sensors.BreakerGenericMagnetometer;
import frc.robot.BreakerLib.devices.sensors.imu.BreakerGenericIMU;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.selftest.SelfTest;

/* CTRE Pigeon IMU 2 implementing the Breaker device interface and Breaker IMU interface. */
public class BreakerPigeon2 extends BreakerGenericIMU implements BreakerGenericMagnetometer {
  private WPI_Pigeon2 pigeon;

  /** Creates a new PigeonIMU 2 object. */
  public BreakerPigeon2(int deviceID) {
    pigeon = new WPI_Pigeon2(deviceID);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
  }

   /** Creates a new PigeonIMU 2 object. */
   public BreakerPigeon2(int deviceID, String busName) {
    pigeon = new WPI_Pigeon2(deviceID, busName);
    deviceName = "Pigeon2_IMU (" + deviceID + ") ";
  }

  @Override
  public double getPitchDegrees() {
    return BreakerMath.angleModulus(pigeon.getPitch());
  }

  @Override
  public double getYawDegrees() {
    return BreakerMath.angleModulus(pigeon.getYaw());
  }

  @Override
  public double getRollDegrees() {
    return BreakerMath.angleModulus(pigeon.getRoll());
  }

  @Override
  public Rotation2d getPitchRotation2d() {
    return Rotation2d.fromDegrees(getPitchDegrees());
  }

  @Override
  public Rotation2d getYawRotation2d() {
    return Rotation2d.fromDegrees(getYawDegrees());
  }

  @Override
  public Rotation2d getRollRotation2d() {
    return Rotation2d.fromDegrees(getRollDegrees());
  }

  @Override
  public Rotation3d getRotation3d() {
      return new Rotation3d(Math.toRadians(getRollDegrees()), Math.toRadians(getPitchDegrees()),  Math.toRadians(getYawDegrees()));
  }

  @Override
  public double[] getRawAngles() {
    double[] RawYPR = new double[3];
    pigeon.getYawPitchRoll(RawYPR);
    return RawYPR;
  }

  @Override
  public double getRawYaw() {
    return getRawAngles()[0];
  }

  @Override
  public double getRawPitch() {
    return getRawAngles()[1];
  }

  @Override
  public double getRawRoll() {
    return getRawAngles()[2];
  }

  /** Does nothing. */
  @Override
  public void setPitch(double value) {
    // TODO Auto-generated method stub

  }

  @Override
  public void setYaw(double value) {
    pigeon.setYaw(0);
  }

  /** Does nothing. */
  @Override
  public void setRoll(double value) {

  }

  /** Sets yaw to 0 */
  @Override
  public void reset() {
    pigeon.setYaw(0);
  }

  public double[] getRawGyroRates() {
    double[] rawRates = new double[3];
    pigeon.getRawGyro(rawRates);
    return rawRates;
  }

  @Override
  public double getRawPitchRate() {
    return getRawGyroRates()[0];
  }

  @Override
  public double getRawRollRate() {
    return getRawGyroRates()[1];
  }

  @Override
  public double getRawYawRate() {
    return getRawGyroRates()[2];
  }

  @Override
  public double getPitchRate() {
    return getRawPitchRate();
  }

  @Override
  public double getYawRate() {
    return getRawYawRate();
  }

  @Override
  public double getRollRate() {
    return getRawRollRate();
  }

  @Override
  public double[] getRawAccelerometerVals() {
    double[] newVals = new double[3];
    for (int i = 0; i < 3; i++) {
      newVals[i] = (BreakerMath.fixedToFloat(getRawAccelerometerValsShort()[i], 14) * 0.000508);
    }
    return newVals;
  }

  public short[] getRawAccelerometerValsShort() {
    short[] accelVals = new short[3];
    pigeon.getBiasedAccelerometer(accelVals);
    return accelVals;
  }

  @Override
    /** @return Unbiased accelerometer x-value in m/s^2. */
    public double getRawAccelX() {
        return (BreakerMath.fixedToFloat(getRawAccelerometerValsShort()[0], 14) * 0.000508);
    }

    @Override
    /** @return Unbiased accelerometer y-value in m/s^2. */
    public double getRawAccelY() {
        return (BreakerMath.fixedToFloat(getRawAccelerometerValsShort()[1], 14) * 0.000508);
    }

    @Override
    /** @return Unbiased accelerometer z-value in m/s^2. */
    public double getRawAccelZ() {
        return (BreakerMath.fixedToFloat(getRawAccelerometerValsShort()[2], 14) * 0.000508);
    }

  /** @return Pigeon's runtime in seconds (max of 255) */
  public int getPigeonUpTime() {

    return pigeon.getUpTime();
  }

  @Override
    public Rotation3d getRawRotation3d() {
        return new Rotation3d(Math.toRadians(getRawAngles()[2]), Math.toRadians(getRawAngles()[1]),
        Math.toRadians(getRawAngles()[0]));
  }

  @Override
  public void runSelfTest() {
    faultStr = null;
    health = DeviceHealth.NOMINAL;
    Pigeon2_Faults curFaults = new Pigeon2_Faults();
    pigeon.getFaults(curFaults);

    if (curFaults.HardwareFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " HARDWARE_FAULT ";
    }
    if (curFaults.MagnetometerFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " MAG_FAULT ";
    }
    if (curFaults.GyroFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += "  GYRO_FAULT ";
    }
    if (curFaults.AccelFault) {
      health = DeviceHealth.INOPERABLE;
      faultStr += " ACCEL_FAULT ";
    }
    if (curFaults.UnderVoltage) {
      health = (health != DeviceHealth.INOPERABLE) ? DeviceHealth.FAULT : health;
      faultStr += " UNDER_6.5V ";
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

  @Override
  public double[] getRawFieldStrenghts() {
      short[] rawShorts = new short[]{3};
      pigeon.getRawMagnetometer(rawShorts);
      return new double[] {(double) rawShorts[0] * 0.6, (double) rawShorts[1] * 0.6, (double) rawShorts[2] * 0.6};
  }

  @Override
  public double[] getBiasedFieldStrenghts() {
      short[] rawShorts = new short[]{3};
      pigeon.getBiasedMagnetometer(rawShorts);
      return new double[] {(double) rawShorts[0] * 0.6, (double) rawShorts[1] * 0.6, (double) rawShorts[2] * 0.6};
  }

  @Override
  public double getCompassFieldStrength() {
      return pigeon.getCompassFieldStrength();
  }

  @Override
  public double getCompassHeading() {
      return MathUtil.angleModulus(pigeon.getCompassHeading());
  }

  @Override
  public double getRawCompassHeading() {
      return pigeon.getCompassHeading();
  }

  @Override
  public Quaternion getQuaternion() {
      double[] quat = new double[4];
      pigeon.get6dQuaternion(quat);
      return new Quaternion(quat[0], quat[1], quat[2], quat[3]);
  }

  @Override
  /** Does nothing. Range is 8 Gs. */
  public void setRange(Range range) {
  }

}
