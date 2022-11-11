// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

/** A class that represents vectorizeable forces (EX: velocity, acceleration, etc) 
 * acting on a 3d object with 6 degrees of freedom (X, Y, and Z linear axise as well as the yaw, pitch, and roll angular axies) */
public class Breaker6AxisForces {
    private BreakerVector3 linearForces;
    private BreakerAngularVector3 angularForces;
    public Breaker6AxisForces(BreakerVector3 linearForces, BreakerAngularVector3 angularForces) {
        this.linearForces = linearForces;
        this.angularForces = angularForces;
    }

    public Breaker6AxisForces() {
        linearForces = new BreakerVector3();
        angularForces = new BreakerAngularVector3();
    }

    public BreakerAngularVector3 getAngularForces() {
        return angularForces;
    }

    public BreakerVector3 getLinearForces() {
        return linearForces;
    }
}
