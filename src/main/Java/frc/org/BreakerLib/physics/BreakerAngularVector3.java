// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

/** Add your docs here. */
public class BreakerAngularVector3 {
    
    private double forceYaw;
    private double forcePitch;
    private double forceRoll;

    public BreakerAngularVector3(double forceYaw, double forcePitch, double forceRoll) {
        this.forceYaw = forceYaw;
        this.forcePitch = forcePitch;
        this.forceRoll = forceRoll;
    }

    public double getForceYaw() {
        return forceYaw;
    }

    public double getForcePitch() {
        return forcePitch;
    }

    public double getForceRoll() {
        return forceRoll;
    }
}
