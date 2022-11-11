// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.movement;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.BreakerLib.physics.Breaker6AxisForces;
import frc.robot.BreakerLib.physics.BreakerAngularVector3;
import frc.robot.BreakerLib.physics.BreakerVector3;

/** Represents an objects 3 dimentional (linear: XYZ / Angular: YPR) position, velocity, acceleration, and jerk at a given time */
public class BreakerMovementState3d {
    private Pose3d position;
    private Breaker6AxisForces velocity;
    private Breaker6AxisForces acceleration;
    private Breaker6AxisForces jerk;

    /**
     * Creates a new BreakerMovementState3d, given a position, velocity, acceleration, and jerk.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     * @param jerk
     */
    public BreakerMovementState3d(Pose3d position, Breaker6AxisForces velocity, Breaker6AxisForces acceleration, Breaker6AxisForces jerk) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
    }

    /**
     * Creates a BreakerMovementState3d with a given position, velocity, and acceleration.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     */
    public BreakerMovementState3d(Pose3d position, Breaker6AxisForces velocity, Breaker6AxisForces acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        jerk = new Breaker6AxisForces(new BreakerVector3(0, 0, 0), new BreakerAngularVector3(0, 0, 0));
    }

     /**
     * Creates a BreakerMovementState3d with a given position, velocity, and acceleration.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     */
    public BreakerMovementState3d(Pose3d position, Breaker6AxisForces velocity) {
        this.position = position;
        this.velocity = velocity;
        acceleration = new Breaker6AxisForces();
        jerk = new Breaker6AxisForces();
    }

    /**
     * @return the position component of this BreakerMovementState3d.
     */
    public Pose3d getPositionComponent() {
        return position;
    }

    /**
     * @return the acceleration component of this BreakerMovementState3d.
     */
    public Breaker6AxisForces getAccelerationComponent() {
        return acceleration;
    }

    /**
     * @return the velocity component of this BreakerMovement3d.
     */
    public Breaker6AxisForces getVelocityComponent() {
        return velocity;
    }

    /**
     * @return returns the jerk component of this BreakerMovementState3d.
     */
    public Breaker6AxisForces getJerkCompoenet() {
        return jerk;
    }
}
