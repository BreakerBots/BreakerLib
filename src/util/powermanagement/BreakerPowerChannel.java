// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.powermanagement;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.BreakerRoboRIO;

/** Add your docs here. */
public class BreakerPowerChannel extends SubsystemBase {
    private int channle = 0;
    private double totalEnergyUsed;
    public BreakerPowerChannel(int channle) {
        this.channle = channle;
    }

    public double getCurrent() {
        return BreakerPowerManager.getPowerDistributor().getCurrent(channle);
    }

    public double getPower() {
        return BreakerPowerManager.getBatteryVoltage() * getCurrent();
    }

    public double getPercentageOfBatteryUsed() {
        return getTotalEnergyUsed() / BreakerPowerManager.getFullBatteryCapacityJoules();
    }

    public double getPercentageOfTotalEnergeyUsed() {
        return getTotalEnergyUsed() / BreakerPowerManager.getTotalEnergyUsed();
    }

    public double getTotalEnergyUsed() {
        return totalEnergyUsed;
    }

    private void intagrateTotalEnergy() {
        totalEnergyUsed += getPower() * BreakerRoboRIO.getInterCycleTimeSeconds();
    }

    public void resetTotalEnergyUsed() {
        totalEnergyUsed = 0;
    }

    @Override
    public void periodic() {
        intagrateTotalEnergy();
    }
}
