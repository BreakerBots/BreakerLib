// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.diff;

import edu.wpi.first.math.controller.RamseteController;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;

/** Add your docs here. */
public class BreakerRamseteConfig {
    private BreakerDiffDrive drivetrain;
    private BreakerGenericOdometer odometer;
    private RamseteController ramseteController;
    public BreakerRamseteConfig(BreakerDiffDrive drivetrain, RamseteController ramseteController) {
        this.drivetrain = drivetrain;
        this.ramseteController = ramseteController;
        odometer = drivetrain;
    }

    public BreakerRamseteConfig(BreakerDiffDrive drivetrain, BreakerGenericOdometer odometer, RamseteController ramseteController) {
        this.drivetrain = drivetrain;
        this.ramseteController  = ramseteController;
        this.odometer = odometer;
    }


    public BreakerDiffDrive getDrivetrain() {
        return drivetrain;
    }

    public BreakerGenericOdometer getOdometer() {
        return odometer;
    }

    public RamseteController getRamseteController() {
        return ramseteController;
    }
}
