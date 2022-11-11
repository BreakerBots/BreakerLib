// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics;

/** A class that represents vectorizeable forces (EX: velocity, acceleration, etc) 
 * acting on a 2d object with 3 degrees of freedom (X, and Y linear axise as well as, the yaw angular axis)*/
public class Breaker3AxisForces {
    private BreakerVector2 linearForces;
    private double angularForce;
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

    public double getAngularForce() {
        return angularForce;
    }

    @Override
    public String toString() {
        return String.format("Breaker3AxisForces(Linear_Forces: %s, Angular_Force: %.2f)", linearForces.toString(), angularForce);
    }
}
