// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.photonvision;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;
import frc.robot.BreakerLib.util.BreakerRoboRIO;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.BreakerUnits;

/**
 * Tracks and identifies pre-defined non-fiducial targets with target data from photon
 * camera.
 */
public class BreakerLegacyPhotonTarget extends SubsystemBase {

    private BreakerPhotonCamera camera;
    private double targetHeightMeters;
    private BreakerGenericOdometer odometryProveider;
    private Pose2d targetLocation;
    private double maxTargetCordinateDeviationMeters;
    private PhotonTrackedTarget assignedTarget;
    private Supplier<PhotonTrackedTarget> assignedTargetSupplier;
    private boolean targetPreAssigned;
    private boolean assignedTargetFound;
    private double targetFoundTimestamp;

    /**
     * Creates a new BreakerPhotonTarget that will activly search for a target that
     * meets its pre-difined paramaters
     * 
     * @param camera                             Photon camera.
     * @param odometryProvider                   BreakerGenericOdometer such as drivetrain or pose estimator for position
     *                                           tracking.
     * @param targetPosition                     Position of target relative to
     *                                           field.
     * @param maxTargetCoordinateDeviationInches Max allowed
     */
    public BreakerLegacyPhotonTarget(BreakerPhotonCamera camera, BreakerGenericOdometer odometryProveider,
            Pose3d targetPosition, double maxTargetCoordinateDeviationMeters) {
        this.camera = camera;
        this.odometryProveider = odometryProveider;
        targetLocation = targetPosition.toPose2d(); // Target location on field as Pose2d.
        this.targetHeightMeters = targetPosition.getZ();
        this.maxTargetCordinateDeviationMeters = maxTargetCoordinateDeviationMeters;
        targetPreAssigned = false;
        assignedTargetFound = false;
    }

    /**
     * Creates a new BreakerPhotonTarget with a given predefined target.
     * 
     * @param camera                 Photon camera.
     * @param assignedTargetSupplier Supplies photon camera target.
     * @param targetHeightInches     Target height from ground.
     */
    public BreakerLegacyPhotonTarget(BreakerPhotonCamera camera, Supplier<PhotonTrackedTarget> assignedTargetSupplier,
            double targetHeightMeters) {
        this.camera = camera;
        this.assignedTargetSupplier = assignedTargetSupplier;
        assignedTarget = assignedTargetSupplier.get();
        assignedTargetFound = (assignedTarget == null) ? false : true;
        this.targetHeightMeters = targetHeightMeters;
    }

    /** Logic used to find a target. */
    private void findAssignedTarget() {
        int runs = 0;
        boolean foundTgt = false;

        if (!targetPreAssigned && camera.hasTargets()) {
            // Loops through tracked targets.
            for (PhotonTrackedTarget prospTgt : camera.getAllRawTrackedTargets()) {
                prospTgt.getCameraToTarget();
                double distanceMeters = PhotonUtils.calculateDistanceToTargetMeters( // Distance from target based on
                                                                                     // constant parameters
                        camera.getCameraHeight(),
                        targetHeightMeters, Math.toRadians(camera.getCameraPitch()),
                        Math.toRadians(prospTgt.getPitch()));

                Translation2d prospTgtTranslation = PhotonUtils.estimateCameraToTargetTranslation(distanceMeters, // 2d
                                                                                                                  // translation
                                                                                                                  // of
                                                                                                                  // target
                                                                                                                  // relative
                                                                                                                  // to
                                                                                                                  // camera
                        Rotation2d.fromDegrees(prospTgt.getYaw()));

                Transform2d prospTgtTransform = PhotonUtils.estimateCameraToTarget(prospTgtTranslation, targetLocation, // 2d transformation from target to camera.
                        odometryProveider.getOdometryPoseMeters().getRotation());

                Pose2d prospTgtPose = odometryProveider.getOdometryPoseMeters().transformBy(prospTgtTransform); // 2d target
                                                                                                         // position
                                                                                                         // relative to
                                                                                                         // field.
                // checks if transform result is close enough to required target location
                if (targetIsCloseEnough(prospTgtPose)) {
                    assignedTarget = prospTgt;
                    foundTgt = true;
                }
                runs++;
            }
            // checks if a target has been successfully found and assigned
            if (runs >= camera.getNumberOfCameraTargets() && foundTgt == true) {
                assignedTargetFound = true;
                targetFoundTimestamp = Timer.getFPGATimestamp();
            } else {
                assignedTargetFound = false;
            }

            // assigns the pre-supplied target if the camera has targets
        } else if (camera.hasTargets()) {
            assignedTarget = assignedTargetSupplier.get();
            assignedTargetFound = (assignedTarget == null) ? false : true;
        }
    }

    /**
     * If prospective target is close enough to actual target position.
     * 
     * @param prospTgtPose Field relative pose of found prospective target
     * @return If prospective target's X and Y coords are within the max deviation.
     */
    private boolean targetIsCloseEnough(Pose2d prospTgtPose) {
        return BreakerMath.isRoughlyEqualTo(prospTgtPose.getX(), targetLocation.getX(),
                maxTargetCordinateDeviationMeters) &&
                BreakerMath.isRoughlyEqualTo(prospTgtPose.getY(), targetLocation.getY(),
                        maxTargetCordinateDeviationMeters);
    }

    /** @return Overall distance from target to camera. */
    public double getTargetDistanceMeters() {
        return PhotonUtils.calculateDistanceToTargetMeters(BreakerUnits.inchesToMeters(camera.getCameraHeight()),
                targetHeightMeters, Math.toRadians(camera.getCameraPitch()),
                Math.toRadians(getPitch()));
    }

    /** @return the relative distances in X and Y between the target and camera */
    public Translation2d getTargetTranslationFromCamera() {
        return PhotonUtils.estimateCameraToTargetTranslation(getTargetDistanceMeters(),
                Rotation2d.fromDegrees(getYaw()));
    }

    /**
     * @return the calculated X and Y cordnates of the target relative to the field
     *         based on vision and odometry
     */
    public Translation2d getTargetTranslationFromField() {
        return odometryProveider.getOdometryPoseMeters().getTranslation().plus(getTargetTranslationFromCamera());
    }

    /**
     * Robot pose based on camera targeting.
     * 
     * @return Pose2d with robot coordinates relative to field.
     */
    public Pose2d getRobotPose() {
        return PhotonUtils.estimateFieldToRobot(
                PhotonUtils.estimateCameraToTarget(getTargetTranslationFromCamera(), targetLocation,
                odometryProveider.getOdometryPoseMeters().getRotation()),
                targetLocation, camera.getCamPositionRelativeToRobot());
    }

    /** Assigned target yaw */
    public double getYaw() {
        return assignedTarget.getYaw();
    }

    /** Assigned target pitch */
    public double getPitch() {
        return assignedTarget.getPitch();
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

    /** If assigned target is found. */
    public boolean getAssignedTargetFound() {
        return assignedTargetFound;
    }

    public double getTargetDataAge() {
        double timediffsec = Timer.getFPGATimestamp() - targetFoundTimestamp;
        return Units.millisecondsToSeconds(camera.getPipelineLatancyMilliseconds()) + timediffsec;
    }

    @Override
    public void periodic() {
        findAssignedTarget();
    }
}
