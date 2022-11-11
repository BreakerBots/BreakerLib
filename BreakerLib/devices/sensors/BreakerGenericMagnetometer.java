// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.sensors;

/** Interface for 3-axis magnitometers with digital compass features  */
public interface BreakerGenericMagnetometer {
    /** X[0], Y[1], Z[2], in microteslas */
   public abstract double[] getRawFieldStrenghts();

   /** X[0], Y[1], Z[2], in microteslas */
   public abstract double[] getBiasedFieldStrenghts();

   /** @return fied strength in microteslas */
   public abstract double getCompassFieldStrength();

   /** angular heding of the compass in +-180 degrees */
   public abstract double getCompassHeading();

   /** angular heading of the compass in degrees, */
   public abstract double getRawCompassHeading();

}
