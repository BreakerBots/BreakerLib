// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BreakerLib.devices.sensors.BreakerPigeon2;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.subsystemcores.drivetrain.swerve.BreakerSwerveDriveConfig;

public class BreakerSwerveDrivePoseEstimateor implements BreakerGenericOdometer {
    private BreakerPigeon2 pigeon2;
    private SwerveDrivePoseEstimator poseEstimator;
    public BreakerSwerveDrivePoseEstimateor(BreakerPigeon2 pigeon2, Pose2d initialPose, BreakerSwerveDriveConfig config, double[] stateModelStanderdDeveation, double gyroStandardDeveation, double[] visionStanderdDeveation) {
        this.pigeon2 = pigeon2;
        poseEstimator = new SwerveDrivePoseEstimator(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), initialPose, config.getKinematics(), 
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(stateModelStanderdDeveation[0], stateModelStanderdDeveation[1], stateModelStanderdDeveation[2]), 
        new MatBuilder<>(Nat.N1(), Nat.N1()).fill(gyroStandardDeveation),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStanderdDeveation[0], visionStanderdDeveation[1], visionStanderdDeveation[2]));
    }

    public Pose2d update(SwerveModuleState... moduleStates) {
        return poseEstimator.update(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), moduleStates);
    }
    
    public Pose2d addVisionMeasurment(Pose2d robotPoseFromVision, double visionPipelineLatencySeconds) {
        poseEstimator.addVisionMeasurement(robotPoseFromVision, Timer.getFPGATimestamp() - visionPipelineLatencySeconds);
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d updateWithVision(Pose2d robotPoseFromVision, double visionPipelineLatencySeconds, SwerveModuleState... moduleStates) {
        update(moduleStates);
        addVisionMeasurment(robotPoseFromVision, visionPipelineLatencySeconds);
        return poseEstimator.getEstimatedPosition();
    }

    public void changeVisionDevs(double visionStrdDevX, double visionStrdDevY, double visionStrdDevTheta) {
        poseEstimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStrdDevX, visionStrdDevY, visionStrdDevTheta));
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


}
