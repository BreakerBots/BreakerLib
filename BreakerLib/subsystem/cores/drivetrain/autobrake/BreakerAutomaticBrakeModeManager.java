// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.autobrake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.BreakerGenericDrivetrain;
import frc.robot.BreakerLib.util.BreakerRoboRIO;

/**
 * Automaticly handles break mode switching for your drivetrain based on the robot's current mode according to
 * given config.
 */
public class BreakerAutomaticBrakeModeManager extends SubsystemBase {

    private boolean brakeInAuto;
    private boolean brakeInTeleop;
    private boolean brakeInTest;
    private boolean brakeInDisabled;
    private boolean autoBrakeIsEnabled;
    private BreakerGenericDrivetrain baseDrivetrain;

    public BreakerAutomaticBrakeModeManager(BreakerAutomaticBrakeModeManagerConfig config) {
        brakeInAuto = config.getBreakInAuto();
        brakeInTeleop = config.getBreakInTeleop();
        brakeInTest = config.getBreakInTest();
        brakeInDisabled = config.getBreakInDisabled();
        baseDrivetrain = config.getBaseDrivetrain();
    }

    public void changeConfig(BreakerAutomaticBrakeModeManagerConfig config) {
        brakeInAuto = config.getBreakInAuto();
        brakeInTeleop = config.getBreakInTeleop();
        brakeInTest = config.getBreakInTest();
        brakeInDisabled = config.getBreakInDisabled();
        baseDrivetrain = config.getBaseDrivetrain();
    }

    public boolean isAutomaticBreakModeEnabled() {
        return autoBrakeIsEnabled;
    }

    public void setAutomaticBreakModeEnabled(Boolean isEnabled) {
        autoBrakeIsEnabled = isEnabled;
    }

    public void setAutomaticBreakMode() {
        switch (BreakerRoboRIO.getCurrentRobotMode()) {
            case AUTONOMOUS:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInAuto);
                break;
            case DISABLED:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInDisabled);
                break;
            case TELEOP:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInTeleop);
                break;
            case TEST:
                baseDrivetrain.setDrivetrainBrakeMode(brakeInTest);
                break;
            case UNKNOWN:
            default:
                baseDrivetrain.setDrivetrainBrakeMode(false);
                break;

        }
    }

    public boolean getBrakeInAuto() {
        return brakeInAuto;
    }

    public boolean getBrakeInDisabled() {
        return brakeInDisabled;
    }

    public boolean getBrakeInTeleop() {
        return brakeInTeleop;
    }

    public boolean getBrakeInTest() {
        return brakeInTest;
    }

    @Override
    public void periodic() {
        if (autoBrakeIsEnabled) {
            setAutomaticBreakMode();
        }
    }
}
