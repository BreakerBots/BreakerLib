// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.powermanagement;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.Map.Entry;

import javax.swing.text.html.parser.Entity;

import edu.wpi.first.hal.simulation.PowerDistributionDataJNI;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.math.averages.BreakerRunningAverage;

/** Add your docs here. */
public class BreakerPowerManager extends SubsystemBase {
    private static PowerDistribution distributor = new PowerDistribution();
    private static double batteryPercentageRemaining;
    private static double batteryJoulesExpended;
    private static double batteryJoulesRemaining;
    private static final double fullBatteryCapacityJoules = 2592000;
    private static final double fullBattetyNominalVoltage = 12.7;
    private static List<BreakerPowerChannel> channelList = BreakerPowerUtil.getNewPowerChannelList(distributor.getType());
    private static TreeMap<BreakerPowerManageable, BreakerPowerManagementConfig> devicesAndConfigs = new TreeMap<>();
    private static boolean activePowerManagementIsEnabled = true;
    private static BreakerRunningAverage runningVoltAverage = new BreakerRunningAverage(250);
    protected static BreakerRunningAverage runningPrecentageAverage = new BreakerRunningAverage(250);
    private BreakerPowerManager manager = new BreakerPowerManager();

    private BreakerPowerManager() {}

    public static void setDistribuionModule(int moduleID, ModuleType moduleType ) {
        distributor = new PowerDistribution(moduleID, moduleType);
        channelList = BreakerPowerUtil.getNewPowerChannelList(moduleType);
    }

    public static void toggleActivePowerManagement(boolean isEnabled) {
        activePowerManagementIsEnabled = isEnabled;
    }

    public static boolean getActitvePowerManagementIsEnabled() {
        return activePowerManagementIsEnabled;
    }

    public static void register(BreakerPowerManageable powerManageableDevice, BreakerPowerManagementConfig config) {
        devicesAndConfigs.put(powerManageableDevice, config);
    }

    public static PowerDistribution getPowerDistributor() {
        return distributor;
    }

    public static double getBatteryVoltage() {
        return distributor.getVoltage();
    }

    public static double getTotalEnergyUsed() {
        return distributor.getTotalEnergy();
    }

    public static void resetTotalEnergyUsed() {
        distributor.resetTotalEnergy();
    }

    public static double getFullBatteryCapacityJoules() {
        return fullBatteryCapacityJoules;
    }

    public static double getFullBattetyNominalvoltage() {
        return fullBattetyNominalVoltage;
    }

    public static BreakerPowerChannel getChannel(int channelNum) {
        return channelList.get(channelNum);
    }

    private static double getRemainingBatteryPercentageJoulesEst() {
        double invPercent = distributor.getTotalEnergy() / fullBatteryCapacityJoules;
        return 1d - invPercent;
    }

    private static double getRemainingBatteryPercentageVoltageEst() {
        runningVoltAverage.addValue(distributor.getVoltage());
        return runningVoltAverage.getAverage() / fullBattetyNominalVoltage;
    }

    // returns the battery's remaing energy as a fractional perentage 0 to 1
    public static double getRemainingBatteryPercentage() {
        double cycleAvg = (getRemainingBatteryPercentageJoulesEst() + getRemainingBatteryPercentageVoltageEst()) / 2;
        return runningPrecentageAverage.addValue(cycleAvg);
    }

    private void managePower() {
        for (Entry<BreakerPowerManageable, BreakerPowerManagementConfig> entry: devicesAndConfigs.entrySet()) {
            if (entry.getKey().isUnderAutomaticControl() && activePowerManagementIsEnabled) {
                entry.getKey().managePower(entry.getValue());
            }
        }
    }

    @Override
    public void periodic() {
        managePower();
    }



}
