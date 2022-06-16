// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

/** A class that represents vectorizeable forces (EX: velocity, acceleration, etc) acting on an object in the X, and Y linear axise as well as, the yaw angular axis*/
public class Breaker3AxisForces {
    BreakerVector2 linearForces;
    double angularForce;
    public Breaker3AxisForces(BreakerVector2 linearForces, double angularForce) {
        this.linearForces = linearForces;
        this.angularForce = angularForce;
    }

    public Breaker3AxisForces() {
        linearForces = new BreakerVector2();
        angularForce = 0;
    }
    
    public BreakerVector2 getLinearForces() {
        return linearForces;
    }

    public double getAngularForces() {
        return angularForce;
    }
}
