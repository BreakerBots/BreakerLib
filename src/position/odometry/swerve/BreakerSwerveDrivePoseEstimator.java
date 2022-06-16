// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.BreakerLib.devices.sensors.BreakerPigeon2;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveDriveConfig;
import frc.robot.BreakerLib.util.math.BreakerMath;

public class BreakerSwerveDrivePoseEstimator implements BreakerGenericOdometer {
    private BreakerPigeon2 pigeon2;
    private BreakerSwerveDriveConfig config;
    private SwerveDrivePoseEstimator poseEstimator;
    private double lastUpdateTimestamp = Timer.getFPGATimestamp();
    private Pose2d prevPose = getOdometryPoseMeters();
    private ChassisSpeeds fieldRelativeChassisSpeeds = new ChassisSpeeds();
    private BreakerMovementState2d prevMovementState = new BreakerMovementState2d();
    private BreakerMovementState2d curMovementState = new BreakerMovementState2d();

    public BreakerSwerveDrivePoseEstimator(BreakerPigeon2 pigeon2, Pose2d initialPose, BreakerSwerveDriveConfig config,
            double[] stateModelStanderdDeveation, double gyroStandardDeveation, double[] visionStanderdDeveation) {
        this.pigeon2 = pigeon2;
        this.config = config;
        poseEstimator = new SwerveDrivePoseEstimator(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), initialPose,
                config.getKinematics(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(stateModelStanderdDeveation[0],
                        stateModelStanderdDeveation[1], stateModelStanderdDeveation[2]),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(gyroStandardDeveation),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStanderdDeveation[0], visionStanderdDeveation[1],
                        visionStanderdDeveation[2]));
    }

    public Pose2d update(SwerveModuleState... moduleStates) {
        prevPose = getOdometryPoseMeters();
        Pose2d pose = poseEstimator.update(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), moduleStates);
        updateChassisSpeeds();
        lastUpdateTimestamp = Timer.getFPGATimestamp();
        return pose;
    }

    public Pose2d addVisionMeasurment(Pose2d robotPoseFromVision, double visionPipelineLatencySeconds) {
        poseEstimator.addVisionMeasurement(robotPoseFromVision,
                Timer.getFPGATimestamp() - visionPipelineLatencySeconds);
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d updateWithVision(Pose2d robotPoseFromVision, double visionPipelineLatencySeconds,
            SwerveModuleState... moduleStates) {
        update(moduleStates);
        addVisionMeasurment(robotPoseFromVision, visionPipelineLatencySeconds);
        return poseEstimator.getEstimatedPosition();
    }

    public void changeVisionDevs(double visionStrdDevX, double visionStrdDevY, double visionStrdDevTheta) {
        poseEstimator.setVisionMeasurementStdDevs(
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStrdDevX, visionStrdDevY, visionStrdDevTheta));
    }

    @Override
    public String toString() {
        return "CURRENT POSE: " + getOdometryPoseMeters().toString();
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        poseEstimator.resetPosition(newPose, Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]));
    }

    @Override
    public Object getBaseOdometer() {
        return poseEstimator;
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return curMovementState;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeChassisSpeeds.vxMetersPerSecond, 
            fieldRelativeChassisSpeeds.vyMetersPerSecond, 
            fieldRelativeChassisSpeeds.omegaRadiansPerSecond, 
            getOdometryPoseMeters().getRotation());
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return fieldRelativeChassisSpeeds;
    }

    private void updateChassisSpeeds() {
        double timeDiff = Timer.getFPGATimestamp() - lastUpdateTimestamp;
        double xSpeed = (getOdometryPoseMeters().getX() - prevPose.getX()) * timeDiff;
        double ySpeed = (getOdometryPoseMeters().getY() - prevPose.getY()) * timeDiff;
        double thetaSpeed = (getOdometryPoseMeters().getRotation().getRadians() - prevPose.getRotation().getRadians()) * timeDiff;
        fieldRelativeChassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
        curMovementState = BreakerMath.movementStateFromChassisSpeedsAndPreviousState(getOdometryPoseMeters(), fieldRelativeChassisSpeeds, timeDiff, prevMovementState);
    }

}
