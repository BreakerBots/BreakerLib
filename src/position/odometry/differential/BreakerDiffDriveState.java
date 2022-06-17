// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.position.odometry.differential;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

/** Add your docs here. */
public class BreakerDiffDriveState {
    private DifferentialDriveWheelSpeeds wheelSpeeds;
    private double leftDriveDistanceMeters;
    private double rightDriveDistanceMeters;
    public BreakerDiffDriveState(double leftDrivetrainSpeedMetersPerSecond, double rightDrivetrainSpeedMetersPerSecond, double leftDriveDistanceMeters, double rightDriveDistanceMeters) {
        wheelSpeeds = new DifferentialDriveWheelSpeeds(leftDrivetrainSpeedMetersPerSecond, rightDrivetrainSpeedMetersPerSecond);
        this.leftDriveDistanceMeters = leftDriveDistanceMeters;
        this.rightDriveDistanceMeters = rightDriveDistanceMeters;
    }

    public BreakerDiffDriveState(DifferentialDriveWheelSpeeds wheelSpeeds, double leftDriveDistanceMeters, double rightDriveDistanceMeters) {
        this.wheelSpeeds = wheelSpeeds;
        this.leftDriveDistanceMeters = leftDriveDistanceMeters;
        this.rightDriveDistanceMeters = rightDriveDistanceMeters;
    }

    public double getLeftDriveDistanceMeters() {
        return leftDriveDistanceMeters;
    }

    public double getRightDriveDistanceMeters() {
        return rightDriveDistanceMeters;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return wheelSpeeds;
    }
}
