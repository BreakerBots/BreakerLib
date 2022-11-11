// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionOdometer;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.differential.BreakerDiffDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDrive;
import frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve.BreakerSwerveDriveConfig;

/** Add your docs here. */
public class BreakerSwerveDriveFiducialVisionPoseEstimator extends SubsystemBase implements BreakerGenericOdometer {
    private BreakerVisionOdometer vision;
    private BreakerSwerveDrive drivetrain;
    private BreakerSwerveDrivePoseEstimator poseEstimator;
    public BreakerSwerveDriveFiducialVisionPoseEstimator(BreakerSwerveDrive drivetrain, BreakerVisionOdometer vision, BreakerSwerveDriveConfig config,
    double[] stateModelStanderdDeveation, double gyroStandardDeveation, double[] visionStanderdDeveation) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        poseEstimator = new BreakerSwerveDrivePoseEstimator(drivetrain.getBaseGyro(), vision.getOdometryPoseMeters(), config, stateModelStanderdDeveation, gyroStandardDeveation, visionStanderdDeveation);
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
        poseEstimator.update(drivetrain.getSwerveModuleStates());
        if (vision.isAnyTargetVisable()) {
            poseEstimator.addVisionMeasurment(vision.getOdometryPoseMeters(), vision.getDataTimestamp());
        }
    }

    @Override
    public void periodic() {
        updateOdometry();
    }
}
