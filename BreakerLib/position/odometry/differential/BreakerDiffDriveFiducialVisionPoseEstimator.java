// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.differential;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;

/** Add your docs here. */
public class BreakerDiffDriveFiducialVisionPoseEstimator extends SubsystemBase implements BreakerGenericOdometer {
    private BreakerVisionOdometer vision;
    private BreakerDiffDrive drivetrain;
    private BreakerDiffDrivePoseEstimator poseEstimator;
    public BreakerDiffDriveFiducialVisionPoseEstimator(BreakerDiffDrive drivetrain, BreakerVisionOdometer vision, double[] stateModelStanderdDeveation, double[] encoderAndGyroStandardDeveation, double[] visionStanderdDeveation) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        poseEstimator = new BreakerDiffDrivePoseEstimator(drivetrain.getBaseGyro(), vision.getOdometryPoseMeters(), stateModelStanderdDeveation, encoderAndGyroStandardDeveation, visionStanderdDeveation);
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        vision.setOdometryPosition(newPose);
        poseEstimator.setOdometryPosition(newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return poseEstimator.getOdometryPoseMeters();
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return poseEstimator.getMovementState();
    }
    
    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return poseEstimator.getRobotRelativeChassisSpeeds();
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return poseEstimator.getFieldRelativeChassisSpeeds();
    }

    private void updateOdometry() {
        poseEstimator.update(drivetrain.getDiffDriveState());
        if (vision.isAnyTargetVisable()) {
            poseEstimator.addVisionMeasurment(vision.getOdometryPoseMeters(), vision.getDataTimestamp());
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
