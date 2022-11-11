// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.management;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.BreakerLib.driverstation.dashboard.BreakerDashboard;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/** Class that manages all avalable autopaths for dashboard chooser */
public class BreakerAutoManager {

    private BreakerAutoPath[] autoPaths;
    private SendableChooser<BreakerAutoPath> selector;
    private BreakerAutoPath doNothingPath = new BreakerAutoPath("Default 'Do Nothing' Path", new SequentialCommandGroup());

    /**
     * Places auto paths onto the dashboard GUI
     * 
     * @param autoPaths Autopaths to select from.
     */
    public BreakerAutoManager(BreakerAutoPath...autoPaths) {
        this.autoPaths = autoPaths;
        selector = new SendableChooser<BreakerAutoPath>();
        selector.setDefaultOption("Do Nothing", doNothingPath);
        for (BreakerAutoPath path: autoPaths) {
            selector.addOption(path.getPathName(), path);
        }
        BreakerDashboard.getSetupTab().add("AUTOPATH SELECTOR", selector).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    /**
     * Constructor with no paths.
     */
    public BreakerAutoManager() {
        autoPaths = new BreakerAutoPath[0];
        selector = new SendableChooser<BreakerAutoPath>();
        selector.setDefaultOption("Do Nothing", doNothingPath);
        BreakerDashboard.getSetupTab().add("AUTOPATH SELECTOR", selector).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    /** Returns auto path selected on the dashboard widget. */
    public BreakerAutoPath getSelected() {
        return selector.getSelected();
    }

    /**Returns base command group autopath from selected BreakerAutoPath. */
    public Command getSelectedAutoPath() {
        BreakerLog.logBreakerLibEvent(" New Autopath Started: " + getSelected().getPathName()); 
        return getSelected().getBaseAutoPath();
    }
}
