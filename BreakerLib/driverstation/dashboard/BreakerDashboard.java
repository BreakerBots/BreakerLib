// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// I SUGGEST USING OBLOG

package frc.robot.BreakerLib.driverstation.dashboard;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/** Add your docs here. */
public class BreakerDashboard {
    private static List<ShuffleboardTab> allTabs = new ArrayList<>();

    public static ShuffleboardTab getMainTab() {
        return getTab("Main");
    }

    public static ShuffleboardTab getSetupTab() {
        return getTab("Setup");
    }
    
    public static ShuffleboardTab getTuningTab() {
        return getTab("Tuning");
    }

    public static ShuffleboardTab getDiagnosticsTab() {
        return getTab("Diagnostics");
    }

    public static ShuffleboardTab getTab(String tabName) {
        ShuffleboardTab tab = Shuffleboard.getTab(tabName);
        if (!allTabs.contains(tab)) {
            allTabs.add(tab);
        }
        return tab;
    }

    public static List<ShuffleboardTab> getAllTabs() {
        return allTabs;
    }
    
}
