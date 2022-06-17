// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.physics.projectilemotion;

import java.util.Vector;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.BreakerLib.devices.sensors.BreakerUltrasonicSensor;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.position.geometry.BreakerPose3d;
import frc.robot.BreakerLib.position.geometry.BreakerRotation3d;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/**
 * Represents a drag affected object's balisitc trajectory without external
 * propulsion (note: force vector matches field cordnate system and is field
 * relative (EX: Z is up))
 */
public class BreakerProjectileTrajectory {
    private BreakerProjectile projectile;
    private BreakerVector3 initialVels;
    private BreakerPose3d launchPoint;

    public BreakerProjectileTrajectory(BreakerProjectile projectile, BreakerVector3 initialVels,
            BreakerPose3d launchPoint) {
        this.projectile = projectile;
        this.initialVels = initialVels;
    }

    /** Max height of projectile in meters. */
    public double getMaxHeight() {
        return (projectile.getTerminalVelSq() / (2 * BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G)) * Math.log(
                (Math.pow(initialVels.getForceZ(), 2) + projectile.getTerminalVelSq()) / projectile.getTerminalVelSq());
    }

    /** Time that a projectile reaches its maximum height. */
    public double getApogeeTime() {
        return (projectile.getTermanalVelMetersPerSec() / BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G)
                * Math.atan(initialVels.getForceZ() / projectile.getTermanalVelMetersPerSec());
    }

    /** 2D position of target at given time, in seconds (TBD) */
    public Translation2d get2dTranslationAtGivenTime(double time) {
        double x = getHorizontalDistInGivenAxisAtGivenTime(time, initialVels.getForceX());
        double y = getHorizontalDistInGivenAxisAtGivenTime(time, initialVels.getForceY());
        return new Translation2d(x, y).plus(launchPoint.get2dPoseComponent().getTranslation());
    }

    /** x = (Vt^2 / g) * ln( (Vt^2 + g * Uo * t) / Vt^2 ) */
    private double getHorizontalDistInGivenAxisAtGivenTime(double time, double initialForce) {
        return (projectile.getTerminalVelSq() / BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G) * Math.log(
                (projectile.getTerminalVelSq() + (BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G * initialForce * time))
                        / projectile.getTerminalVelSq());
    }

    private double getTimeAtGivenDistance(double distance, double initialForce) {
        double exp = Math.exp(distance / (projectile.getTerminalVelSq() / BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G));
        double vtExp = (projectile.getTerminalVelSq() * exp) - projectile.getTerminalVelSq();
        return vtExp / BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G / initialForce;
    }

    public double getTimeOfFightToTarget(Translation2d targedLocation) {
        return getTimeAtGivenDistance(targedLocation.getDistance(launchPoint.get2dPoseComponent().getTranslation()),
                Math.hypot(initialVels.getForceX(), initialVels.getForceY()));
    }

    private double getVelInGivenAxisAtGivenTime(double time, double initialForce) {
        return projectile.getTerminalVelSq() * initialForce
                / (projectile.getTerminalVelSq() + BreakerUnits.METERS_PER_SECOND_SQUARED_IN_G * initialForce * time);
    }

    public BreakerVector3 getForceVectorAtGivenTime(double time) {
        double x = getVelInGivenAxisAtGivenTime(time, initialVels.getForceX());
        double y = getVelInGivenAxisAtGivenTime(time, initialVels.getForceY());
        double z = getVelInGivenAxisAtGivenTime(time, initialVels.getForceZ());
        return new BreakerVector3(x, y, z);
    }

    /** Creates a corrected 2D pose for the target (I.E. where you should aim) based on chassis movement. */
    public Translation2d getMovingLaunchCorrectionAsNewTargetLocation(ChassisSpeeds fieldRelativeSpeeds,
            Translation2d targetLocation) {
        BreakerProjectileTrajectory trajectory = new BreakerProjectileTrajectory(projectile,
                new BreakerVector3(initialVels.getForceX() + fieldRelativeSpeeds.vxMetersPerSecond,
                        initialVels.getForceY() + fieldRelativeSpeeds.vyMetersPerSecond, initialVels.getForceZ()),
                launchPoint);
        Translation2d correctedLocation = targetLocation
                .minus(trajectory.get2dTranslationAtGivenTime(getTimeOfFightToTarget(targetLocation)));
        return targetLocation.plus(correctedLocation);
    }

    /** Creates a corrected 3D vector for the projectile (I.E. corrected launch forces) based on movement of chassis. */
    public BreakerVector3 getMovingLaunchCorrectionAsNewLaunchForces(ChassisSpeeds fieldRelativeSpeeds,
            Translation2d targetLocation) {
        BreakerProjectileTrajectory trajectory = new BreakerProjectileTrajectory(projectile,
                new BreakerVector3(initialVels.getForceX() + fieldRelativeSpeeds.vxMetersPerSecond,
                        initialVels.getForceY() + fieldRelativeSpeeds.vyMetersPerSecond, initialVels.getForceZ()),
                launchPoint);
        double baseExpectedImpactTime = getTimeOfFightToTarget(targetLocation);
        BreakerVector3 baseExpectedImpactVec = getForceVectorAtGivenTime(baseExpectedImpactTime);
        BreakerVector3 predictedImpactVec = trajectory.getForceVectorAtGivenTime(baseExpectedImpactTime);
        BreakerVector3 correctionVec = baseExpectedImpactVec.minus(predictedImpactVec);
        return initialVels.plus(correctionVec);
    }
}
