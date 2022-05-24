// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.differential;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.BreakerLib.devices.sensors.BreakerPigeon2;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;

/** Add your docs here. */
public class BreakerDiffPoseEstimator implements BreakerGenericOdometer {
    private BreakerPigeon2 pigeon2;
    private DifferentialDrivePoseEstimator poseEstimator;
    private Pose2d currentPose;
    public BreakerDiffPoseEstimator(BreakerPigeon2 pigeon2, Pose2d initialPose, double[] stateModelStanderdDeveation, double[] encoderAndGyroStandardDeveation, double[] visionStanderdDeveation) {
        currentPose = initialPose;
        this.pigeon2 = pigeon2;
        poseEstimator = new DifferentialDrivePoseEstimator(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), initialPose,
            new MatBuilder<>(Nat.N5(), Nat.N1()).fill(stateModelStanderdDeveation[0], stateModelStanderdDeveation[1], stateModelStanderdDeveation[2], stateModelStanderdDeveation[3], stateModelStanderdDeveation[4]), // State measurement standard deviations. X, Y, theta., 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(encoderAndGyroStandardDeveation[0], encoderAndGyroStandardDeveation[1], encoderAndGyroStandardDeveation[2]), // Local measurement standard deviations. Left encoder, right encoder, gyro.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStanderdDeveation[0], visionStanderdDeveation[1], visionStanderdDeveation[2])); // Global measurement standard deviations. X, Y, and theta.4
    }

    public Pose2d update(BreakerDiffDriveState currentDriveState) {
        currentPose = poseEstimator.update(Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]), currentDriveState.getWheelSpeeds(), currentDriveState.getLeftDriveDistanceMeters(), currentDriveState.getRightDriveDistanceMeters());
        return currentPose;
    }

    public Pose2d addVisionMeasurment(Pose2d robotPoseFromVision, double visionPipelineLatencySeconds) {
        poseEstimator.addVisionMeasurement(robotPoseFromVision, Timer.getFPGATimestamp() - visionPipelineLatencySeconds);
        currentPose = poseEstimator.getEstimatedPosition();
        return currentPose;
    }

    public void changeVisionDevs(double visionStrdDevX, double visionStrdDevY, double visionStrdDevTheta) {
        poseEstimator.setVisionMeasurementStdDevs(new MatBuilder<>(Nat.N3(), Nat.N1()).fill(visionStrdDevX, visionStrdDevY, visionStrdDevTheta));
    }

    public Pose2d updateWithVision(BreakerDiffDriveState currentDriveState, Pose2d robotPoseFromVision, double visionPipelineLatencySeconds) {
        update(currentDriveState);
        addVisionMeasurment(robotPoseFromVision, visionPipelineLatencySeconds);
        currentPose = poseEstimator.getEstimatedPosition();
        return currentPose;
    }

    public void setOdometryPosition(Pose2d newPose) {
        poseEstimator.resetPosition(newPose, Rotation2d.fromDegrees(pigeon2.getRawAngles()[0]));
    }

    @Override
    public String toString() {
        return "CURRENT POSE: " + getOdometryPoseMeters().toString();
    }

    @Override
    public Object getBaseOdometer() {
        return poseEstimator;
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        currentPose = poseEstimator.getEstimatedPosition();
        return currentPose;
    }
}
