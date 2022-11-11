// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

/** Add your docs here. */
public class BreakerAngularVector3 {

    private double magnatudeYaw;
    private double magnatudePitch;
    private double magnatudeRoll;

    public BreakerAngularVector3(double magnatudeYaw, double magnatudePitch, double magnatudeRoll) {
        this.magnatudeYaw = magnatudeYaw;
        this.magnatudePitch = magnatudePitch;
        this.magnatudeRoll = magnatudeRoll;
    }

    public BreakerAngularVector3() {
        this.magnatudeYaw = 0;
        this.magnatudePitch = 0;
        this.magnatudeRoll = 0;
    }

    public double getMagnatudeYaw() {
        return magnatudeYaw;
    }

    public double getMagnaudePitch() {
        return magnatudePitch;
    }

    public double getMagnatudeRoll() {
        return magnatudeRoll;
    }
}
