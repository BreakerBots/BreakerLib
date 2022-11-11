// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photonvision;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Fiducial marker target. Use if using AprilTags w/ 3D calibrated camera. */
public class BreakerFiducialPhotonTarget extends SubsystemBase {
    private PhotonTrackedTarget assignedTarget;
    private double lastDataUpdate = Timer.getFPGATimestamp();

    private BreakerPhotonCamera camera;
    private BreakerPhotonCamera[] cameras;
    private Pose3d targetPose;
    private boolean assignedTargetFound = false;
    private boolean assignedTargetFoundInCycle = false; 
    private int fiducialID;

    /**
     * 
     * @param fiducialID - The numeric ID of the Fidicial Marker, this is tied to the tag's pattern and is constant
     * @param targetPose - The 3 dimensional orientation of the target relative the the field origin
     * @param cameras - Any and all cameras that will be used to track the target
     */
    public BreakerFiducialPhotonTarget(int fiducialID, Pose3d targetPose, BreakerPhotonCamera... cameras) {
        this.targetPose = targetPose;
        this.fiducialID = fiducialID;
        this.cameras = cameras;
    }

    /** Function for finding best assigned fiducial target. */
    private void findAssignedFiducial() {
        assignedTargetFoundInCycle = false;
        PhotonTrackedTarget bestTgt = assignedTarget;
        BreakerPhotonCamera bestCam = camera;
        for (BreakerPhotonCamera cam: cameras) { // Loops through all cameras
            if (cam.hasTargets()) {
                for (PhotonTrackedTarget prospTgt: cam.getAllRawTrackedTargets()) { // Goes through all targets found
                    if (prospTgt.getFiducialId() == fiducialID) {
                        assignedTargetFound = true;
                        if (!assignedTargetFoundInCycle) { // runs only if bestTgt has not been assigned this cycle
                            bestTgt = prospTgt;
                            bestCam = cam;
                            assignedTargetFoundInCycle = true;
                        } else if (prospTgt.getPoseAmbiguity() < bestTgt.getPoseAmbiguity()) { // runs only if bestTgt has been assigned this cycle, checks if new tgt is better
                            bestTgt = prospTgt;
                            bestCam = cam;
                        }
                        lastDataUpdate = Timer.getFPGATimestamp(); // used for data age, shoud only be assigned once per cycle
                    }
                }
            }
        }
        assignedTarget = bestTgt;
        camera = bestCam;
    }

    /** @return Timestamp of last target found. */
    public double getLastTargetFoundTimestamp() {
        return lastDataUpdate;
    }

    /** @return Latency of target data in seconds. */
    public double getTargetDataAge() {
        double timediffsec = Timer.getFPGATimestamp() - lastDataUpdate;
        return Units.millisecondsToSeconds(camera.getPipelineLatancyMilliseconds()) + timediffsec;
    }

    /** @return The timestamp of the vision data */
    public double getTargetDataTimestamp() {
        return Timer.getFPGATimestamp() - getTargetDataAge();
    }

    /** @return 3d pose of camera. */
    public Pose3d getCameraPose3d() {
        return targetPose.transformBy(assignedTarget.getCameraToTarget().inverse());
    }

    /** @return 2d pose of camera. */
    public Pose2d getCameraPose() {
        return getCameraPose3d().toPose2d();
    }

    /** @return 3d pose of robot from target data. */
    public Pose3d getRobotPose3d() {
        return getCameraPose3d().transformBy(camera.get3dCamPositionRelativeToRobot().inverse());
    }

    /** @return 2d pose of robot from target data. */
    public Pose2d getRobotPose() {
        return getRobotPose3d().toPose2d();
    }

    /** @return Assigned target camera relative yaw */
    public double getYaw() {
        return assignedTarget.getYaw();
    }

    /** @return Assigned target camera relative pitch */
    public double getPitch() {
        return assignedTarget.getPitch();
    }
 
    /** @return Target's yaw relative to robot's zero point. */
    public double getRobotRelativeYaw() {
        return Rotation2d.fromDegrees(getYaw()).minus(camera.getCamPositionRelativeToRobot().getRotation()).getDegrees();
    }

    /** @return Target's pitch relative to robot's zero point. */
    public double getRobotRelativePitch() {
        return Rotation2d.fromDegrees(getYaw()).minus(new Rotation2d(camera.get3dCamPositionRelativeToRobot().getRotation().getY())).getDegrees();
    }

    /** Assigned target skew */
    public double getSkew() {
        return assignedTarget.getSkew();
    }

    /** Assigned target area */
    public double getArea() {
        return assignedTarget.getArea();
    }

    /** List of target corner coordinates. */
    public List<TargetCorner> getTargetCorners() {
        return assignedTarget.getCorners();
    }

    /** If assigned target has been found at any point during operation */
    public boolean getAssignedTargetFound() {
        return assignedTargetFound;
    }

    /** If assigned target was found in the most recent cycle */
    public boolean isAssignedTargetVisible() {
        return assignedTargetFound && assignedTargetFoundInCycle;
    }

    /** @return Distance from camera to target. */
    public double getDistance() {
        return assignedTarget.getCameraToTarget().getTranslation().toTranslation2d().getNorm();
    }

    /** @return ID of fiducial target */
    public int getFiducialID() {
        return fiducialID;
    }

    /** @return Ambiguity of pose, from 0 to 1. 0 = most accurate, 1 = least accurate. Anything above 0.2 is likely inaccurate */
    public double getPoseAmbiguity() {
        return assignedTarget.getPoseAmbiguity();
    }

    @Override
    public void periodic() {
        findAssignedFiducial();
    }
}
