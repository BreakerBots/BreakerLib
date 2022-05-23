// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

/** Represents a 3d transformation of a pose. */
public class BreakerTransform3d {

    private BreakerTranslation3d translation;
    private BreakerRotation3d rotation;

    /**
     * Creates a new BreakerTransform3d with a translation and rotation.
     * 
     * @param translation Positional values of transformation.
     * @param rotation    Rotational values of transformation.
     */
    public BreakerTransform3d(BreakerTranslation3d translation, BreakerRotation3d rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    /**
     * Creates a new BreakerTransform3d based on a pose.
     * 
     * @param pose 3d pose of transformation.
     */
    public BreakerTransform3d(BreakerPose3d pose) {
        this.translation = pose.getTranslationComponent();
        this.rotation = pose.getRotationComponent();
    }

    public BreakerRotation3d getRotationComponent() {
        return rotation;
    }

    public BreakerTranslation3d getTranslationComponent() {
        return translation;
    }

    /** 2d transformation of x-pos, y-pos, and yaw. */
    public Transform2d get2dTransformationComponent() {
        return new Transform2d(translation.get2dTranslationComponent(), rotation.getYaw());
    }

    /**
     * Transforms 3d pose by its own transform value.
     * 
     * @param pose2d 2d pose. Pitch, roll, and z-position are equal to 0.
     * @return 3d pose based on given 2d pose transformed by this
     *         BreakerTransformation3d.
     */
    public BreakerPose3d transformFromPose2d(Pose2d pose2d) {
        return new BreakerPose3d(pose2d).transformBy(this);
    }
}
