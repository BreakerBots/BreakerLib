// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.vision.standardlimelight;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.BreakerLib.util.math.BreakerMath;

/** Add your docs here. */
public class BreakerLimelightTarget {
    private int pipelineNum;
    private double targetHeight;
    private BreakerLimelight limelight;
    private double targetOffsetDistX = 0;
    private double targetOffsetDistY = 0;
    private double targetOffsetDistZ = 0;
    private boolean targetOffsetHasBeenSet = false;

    /** Creates the profile for the target of a limelight
     * @param targetHeight the height of the intended target form the ground (refers to the vision target being tracked)
     * @param limelight the limelight used to track the intended target (refers to the vision target being tracked)
     * @param pipelineNum the pipeline number (0 - 9) that the limelight needs to use to track the intended target
     */
    public BreakerLimelightTarget(double targetHeight, BreakerLimelight limelight, double pipelineNum) {
      targetHeight = this.targetHeight;
      pipelineNum = this.pipelineNum;
      limelight = this.limelight;
    }

    public void setOffsetDistanceFromVisionTarget(double offsetX, double offsetY, double offsetZ) {
      targetOffsetDistX = offsetX;
      targetOffsetDistY = offsetY;
      targetOffsetDistZ = offsetZ;
      targetOffsetHasBeenSet = true;
    }

    public double getTargetPipeline() {
      return pipelineNum;
    }

    public double getRawTargetOffsetX() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("tx").getDouble(0);
    }

    public double getTargetOffsetX() {
      double x = Math.pow(targetOffsetDistX, 2) - Math.pow(getRawTargetDistance(), 2) - Math.pow(getTargetDistance(), 2);
      double y = (((x / -2) / getRawTargetDistance()) / getTargetDistance());
      return (targetOffsetHasBeenSet ? (getRawTargetOffsetX() + Math.toDegrees(Math.acos(y))) : getRawTargetOffsetX());
    }

    public double getRawTargetOffsetY() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("ty").getDouble(0);
    }

    public double getTargetOffsetY() {
      double newHeight = (targetHeight - limelight.getMountingHeight()) + targetOffsetDistY;
      return (targetOffsetHasBeenSet ? (Math.toDegrees(Math.atan((newHeight / getTargetDistance())))) : getRawTargetOffsetY());
    }

    public double getTargetArea() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("ta").getDouble(0);
    }

    public double getTargetSkew() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("ts").getDouble(0);
    }

    public double get3DTargetData() {
      return NetworkTableInstance.getDefault().getTable(limelight.getName()).getEntry("camtran").getDouble(0);
    }

    public double getRawTargetDistance() {
      double camRelHeightTgt = targetHeight - limelight.getMountingHeight();
      double corTarAng = limelight.getMountingAngle() + getRawTargetOffsetY();
      return (camRelHeightTgt / Math.tan(corTarAng));
    }

    public double getTargetDistance() {
      return (targetOffsetHasBeenSet ? (BreakerMath.getHypotenuse((getRawTargetDistance() + targetOffsetDistZ), targetOffsetDistX)) : getRawTargetDistance());
    }  
  }
