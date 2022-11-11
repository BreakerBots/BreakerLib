// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.movement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.BreakerLib.physics.Breaker3AxisForces;

/** Represents an object's 2D (linear: XY / Angular: Y) position (m & rad), velocity (m & rad/s), acceleration(m & rad/s^2), and jerk (m & rad /s^3) at a given time. */
public class BreakerMovementState2d {
    private Pose2d position;
    private Breaker3AxisForces velocity;
    private Breaker3AxisForces acceleration;
    private Breaker3AxisForces jerk;

    /**
     * Creates a new BreakerMovementState2d using a given position, velocity, acceleration, and jerk.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     * @param jerk
     */
    public BreakerMovementState2d(Pose2d position, Breaker3AxisForces velocity, Breaker3AxisForces acceleration, Breaker3AxisForces jerk) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
    }

    /**
     * Creates a new BreakerMovementState2d using a given position, velocity and acceleration.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     */
    public BreakerMovementState2d(Pose2d position, Breaker3AxisForces velocity, Breaker3AxisForces acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        jerk = new Breaker3AxisForces();
    }

        /**
     * Creates a new BreakerMovementState2d using a given position and velocity.
     * 
     * @param position
     * @param velocity
     */
    public BreakerMovementState2d(Pose2d position, Breaker3AxisForces velocity) {
        this.position = position;
        this.velocity = velocity;
        acceleration = new Breaker3AxisForces();
        jerk = new Breaker3AxisForces();
    }

     /**
     * Creates a new BreakerMovementState2d and uses default values for the position, velocity, acceleration, and jerk.
     * 
     * @param position
     * @param velocity
     * @param acceleration
     */
    public BreakerMovementState2d() {
        position = new Pose2d();
        velocity = new Breaker3AxisForces();
        acceleration = new Breaker3AxisForces();
        jerk = new Breaker3AxisForces();
    }

    public Pose2d estimateFuturePose(double deltaTimeSeconds) {
        double accelX = acceleration.getLinearForces().getMagnatudeX() + (jerk.getLinearForces().getMagnatudeX() * deltaTimeSeconds);
        double accelY = acceleration.getLinearForces().getMagnatudeY() + (jerk.getLinearForces().getMagnatudeY() * deltaTimeSeconds);
        double accelT = acceleration.getAngularForce() + (jerk.getAngularForce() * deltaTimeSeconds);
        double velX = velocity.getLinearForces().getMagnatudeX() + (accelX * deltaTimeSeconds);
        double velY = velocity.getLinearForces().getMagnatudeY() + (accelY * deltaTimeSeconds);
        double velT = velocity.getAngularForce() + (accelT * deltaTimeSeconds);
        double x = position.getX() + (velX * deltaTimeSeconds);
        double y = position.getY() + (velY * deltaTimeSeconds);
        double t = position.getRotation().getRadians() + (velT * deltaTimeSeconds);
        return new Pose2d(x, y, new Rotation2d(t));
    }

    /**
     * @return The position component of this BreakerMovementState2d
     */
    public Pose2d getPositionComponent() {
        return position;
    }

    /**
     * @return The acceleration component of this BreakerMovementState2d.
     */
    public Breaker3AxisForces getAccelerationComponent() {
        return acceleration;
    }
    /**
     * @return The velocity component of this BreakerMovementState2d.
     */
    public Breaker3AxisForces getVelocityComponent() {
        return velocity;
    }

    /**
     * @return The jerk component of this BreakerMovementState2d.
     */
    public Breaker3AxisForces getJerkCompoenet() {
        return jerk;
    }

    @Override
    public String toString() {
        return String.format("Breaker3AxisForces(Position: %s, Velocity: %s, Acceleration: %s, Jerk: %s)", position, velocity, acceleration, jerk);
    }
}
