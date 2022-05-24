// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation;

import edu.wpi.first.math.controller.PIDController;

/** Why do I exist? */
public class BreakerTunerPID {
    private PIDController controller;
    private boolean usesPassedInController;
    private double kP;
    private double kI;
    private double kD;

    public BreakerTunerPID(PIDController controller) {
        this.controller = controller;
        usesPassedInController = true;
    }

    public BreakerTunerPID() {
        usesPassedInController = false;
    }

    private void createWidgit() {
        
    }


}
