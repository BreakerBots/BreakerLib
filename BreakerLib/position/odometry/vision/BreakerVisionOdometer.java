// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.vision;

import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;

/** Add your docs here. */
public class BreakerVisionOdometer extends SubsystemBase implements BreakerGenericOdometer {
    private BreakerVisionPoseFilter visionPoseFilter;
    private Pose2d curPose;
    private Transform2d offset;
    public BreakerVisionOdometer(BreakerVisionPoseFilter visionPoseFilter) {
        this.visionPoseFilter = visionPoseFilter;
        curPose = new Pose2d();
        offset = new Transform2d();
    }

    public double getDataTimestamp() {
        return visionPoseFilter.getDataTimestamp();
    }

    public boolean isAnyTargetVisable() {
        return visionPoseFilter.isAnyTargetVisable();
    }

    @Override
    public void setOdometryPosition(Pose2d newPose) {
        offset = curPose.minus(newPose);
    }

    @Override
    public Pose2d getOdometryPoseMeters() {
        return curPose;
    }

    @Override
    public BreakerMovementState2d getMovementState() {
        return null;
    }

    @Override
    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        // TODO Auto-generated method stub
        return null;
    }

    private void updateChassisSpeeds() {}

    private void updatePose() {
        Pose2d pos = visionPoseFilter.getFilteredRobotPose();
        curPose = pos.transformBy(offset);
    }

    @Override
    public void periodic() {
        updatePose();
    }
}
