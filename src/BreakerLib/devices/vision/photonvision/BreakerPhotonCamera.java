// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.BreakerLib.position.geometry.BreakerTransform3d;

/** Photon camera */
public class BreakerPhotonCamera {

    private PhotonCamera camera;
    private double cameraAngle; // Mounting angle
    private double cameraHeightIns; // Height relative to ground. MAKE THIS METERS?
    private double verticalFOV;
    private double horizontalFOV;
    private BreakerTransform3d cameraPositionRelativeToRobot; // Height relative to ground, all else relative to robot
                                                              // position.

    /**
     * Creates a new camera that uses a PhotonVision-based computer vision
     * algorithem
     * 
     * @param cameraName                    Name of camera used to retreive data.
     * @param verticalFOV                   Vertical field of view.
     * @param horizontalFOV                 Horizontal field of view.
     * @param cameraPositionRelativeToRobot Transformation between robot center and
     *                                      camera position (Z translation is
     *                                      relative to the ground).
     */
    public BreakerPhotonCamera(String cameraName, double verticalFOV, double horizontalFOV,
            BreakerTransform3d cameraPositionRelativeToRobot) {
        camera = new PhotonCamera(cameraName);
        this.cameraPositionRelativeToRobot = cameraPositionRelativeToRobot;
        this.cameraAngle = cameraPositionRelativeToRobot.getRotationComponent().getPitch().getDegrees();
        this.cameraHeightIns = Units
                .metersToInches(cameraPositionRelativeToRobot.getTranslationComponent().getMetersZ());
        this.verticalFOV = verticalFOV;
        this.horizontalFOV = horizontalFOV;

    }

    /** Overall raw result from photon camera. */
    public PhotonPipelineResult getLatestRawResult() {
        return camera.getLatestResult();
    }

    /** If camera is locked onto any targets. */
    public boolean hasTargets() {
        return getLatestRawResult().hasTargets();
    }

    /**
     * Returns a list of all raw PhotonTrackedTargets the camera has in its field of
     * view
     */
    public PhotonTrackedTarget[] getAllRawTrackedTargets() {
        return getLatestRawResult().targets.toArray(new PhotonTrackedTarget[getLatestRawResult().targets.size()]);
    }

    /** Number of camera targets currently locked on. */
    public int getNumberOfCameraTargets() {
        return getAllRawTrackedTargets().length;
    }

    /** Camera latency in milliseconds */
    public double getPipelineLatancyMilliseconds() {
        return getLatestRawResult().getLatencyMillis();
    }

    /** Sets camera pipeline based on given number. */
    public void setPipelineNumber(int pipeNum) {
        camera.setPipelineIndex(pipeNum);
    }

    /** Returns the index of the active pipeline on the camera as an int */
    public int getCurrentPipelineNumber() {
        return camera.getPipelineIndex();
    }

    /**
     * Returns the raw PhotonTrackedTarget object representing the best tracked
     * target according to the pipeline's native sort
     */
    public PhotonTrackedTarget getBestTarget() {
        return getLatestRawResult().getBestTarget();
    }

    /** Height is relative to ground. */
    public double getCameraHeightIns() {
        return cameraHeightIns;
    }

    /** Returns pitch of camera. */
    public double getCameraAngle() {
        return cameraAngle;
    }

    public double getVerticalFOV() {
        return verticalFOV;
    }

    public double getHorizontalFOV() {
        return horizontalFOV;
    }

    /** 2d pose of camera relative to robot. */
    public Transform2d getCamPositionRelativeToRobot() {
        return cameraPositionRelativeToRobot.get2dTransformationComponent();
    }

    public void updateCamPositionRelativeToRobot(BreakerTransform3d newTransform) {
        cameraPositionRelativeToRobot = newTransform;
        cameraAngle = newTransform.getRotationComponent().getPitch().getDegrees();
        cameraHeightIns = Units.metersToInches(newTransform.getTranslationComponent().getMetersZ());
    }

}
