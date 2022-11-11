// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.autobrake;

import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;

/** Config for {@link BreakerAutomaticBrakeModeManager}. */
public class BreakerAutomaticBrakeModeManagerConfig {
    private boolean breakInAuto;
    private boolean breakInTeleop;
    private boolean breakInTest;
    private boolean breakInDisabled;
    private BreakerGenericDrivetrain baseDrivetrain;
    public BreakerAutomaticBrakeModeManagerConfig(BreakerGenericDrivetrain baseDrivetrain, boolean breakInTeleop, boolean breakInAuto, boolean breakInTest, boolean breakInDisabled) {
        this.breakInAuto = breakInAuto;
        this.breakInDisabled = breakInDisabled;
        this.breakInTeleop = breakInTeleop;
        this.breakInTest = breakInTest;
        this.baseDrivetrain = baseDrivetrain;
    }

    /** Team 5104 default configuration, with all braking disabled allong with the robot for ease of transport but enabled outherwise */
    public BreakerAutomaticBrakeModeManagerConfig(BreakerGenericDrivetrain baseDrivetrain) {
        this.breakInAuto = true;
        this.breakInDisabled = false;
        this.breakInTeleop = true;
        this.breakInTest = true;
        this.baseDrivetrain = baseDrivetrain;
    }

    public boolean getBreakInAuto() {
        return breakInAuto;
    }

    public boolean getBreakInDisabled() {
        return breakInDisabled;
    }

    public boolean getBreakInTeleop() {
        return breakInTeleop;
    }

    public boolean getBreakInTest() {
        return breakInTest;
    }

    public BreakerGenericDrivetrain getBaseDrivetrain() {
        return baseDrivetrain;
    }
}
