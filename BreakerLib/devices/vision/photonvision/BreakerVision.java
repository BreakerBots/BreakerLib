// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photonvision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.BreakerLib.position.odometry.vision.BreakerVisionPoseFilter;

/** WIP */
public class BreakerVision {
    private BreakerFiducialPhotonTarget[] targets;
    private BreakerPhotonCamera[] cameras;
    private BreakerVisionPoseFilter poseFilter;
    public BreakerVision(double poseFilterTrustCoef, double poseFilterMaxUncertanty, BreakerPhotonCamera[] cameras, Pair<Integer, Pose3d>[] fiducialTargetIDsAndPoses) {
        targets = new BreakerFiducialPhotonTarget[fiducialTargetIDsAndPoses.length];
        this.cameras = cameras;
        for (int i = 0; i < fiducialTargetIDsAndPoses.length; i++) {
            Pair<Integer, Pose3d> dataPair = fiducialTargetIDsAndPoses[i];
            targets[i] = new BreakerFiducialPhotonTarget(dataPair.getFirst(), dataPair.getSecond(), cameras);
        }
        poseFilter = new BreakerVisionPoseFilter(poseFilterTrustCoef, poseFilterMaxUncertanty, targets);
    }

    public BreakerPhotonCamera[] getCameras() {
        return cameras;
    }

    public BreakerPhotonCamera getCamera(String cameraName) {
        for (BreakerPhotonCamera cam: cameras) {
            if (cam.getDeviceName() == cameraName) {
                return cam;
            }
        }
        return null;
    }

    public boolean hasTargets() {
        for (BreakerPhotonCamera cam: cameras) {
            if (cam.hasTargets()) {
                return true;
            }
        }
        return false;
    }

    public BreakerFiducialPhotonTarget[] getTargets() {
        return targets;
    }

    public Pose3d getFilteredRobotPose3d() {
        return poseFilter.getFilteredRobotPose3d();
    }

    public Pose2d getFilteredRobotPose() {
        return poseFilter.getFilteredRobotPose();
    }
}
