// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.movement;

import frc.robot.BreakerLib.physics.Breaker6AxisForces;
import frc.robot.BreakerLib.physics.BreakerAngularVector3;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.position.geometry.BreakerPose3d;

/** Represents an objects 3 dimentional (linear: XYZ / Angular: YPR) position, velocity, acceleration, and jerk at a given time */
public class BreakerMovementState3d {
    BreakerPose3d position;
    Breaker6AxisForces velocity;
    Breaker6AxisForces acceleration;
    Breaker6AxisForces jerk;
    public BreakerMovementState3d(BreakerPose3d position, Breaker6AxisForces velocity, Breaker6AxisForces acceleration, Breaker6AxisForces jerk) {
        this.position = position;
        this.acceleration = acceleration;
        this.jerk = jerk;
    }

    public BreakerMovementState3d(BreakerPose3d position, Breaker6AxisForces velocity, Breaker6AxisForces acceleration) {
        this.position = position;
        this.acceleration = acceleration;
        jerk = new Breaker6AxisForces(new BreakerVector3(0, 0, 0), new BreakerAngularVector3(0, 0, 0));
    }

    public BreakerPose3d getPositionComponent() {
        return position;
    }

    public Breaker6AxisForces getAccelerationComponent() {
        return acceleration;
    }

    public Breaker6AxisForces getVelocityComponent() {
        return velocity;
    }

    public Breaker6AxisForces getJerkCompoenet() {
        return jerk;
    }
}
