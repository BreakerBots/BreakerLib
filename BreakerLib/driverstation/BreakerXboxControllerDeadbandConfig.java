// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation;

/** Add your docs here. */
public class BreakerXboxControllerDeadbandConfig {
    private double leftX = 0.0, leftY = 0.0, rightX = 0.0, rightY = 0.0, leftTriggerAxis = 0.0, rightTriggerAxis = 0.0;
    public BreakerXboxControllerDeadbandConfig(double leftX, double leftY, double rightX, double rightY, double leftTriggerAxis, double rightTriggerAxis) {
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;
        this.leftTriggerAxis = leftTriggerAxis;
        this.rightTriggerAxis = rightTriggerAxis;
    }

    public BreakerXboxControllerDeadbandConfig(double leftX, double leftY, double rightX, double rightY) {
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;
        leftTriggerAxis = 0.0;
        rightTriggerAxis = 0.0;
    }

    public BreakerXboxControllerDeadbandConfig() {}
    
    public double getLeftTriggerAxis() {
        return leftTriggerAxis;
    }

    public double getLeftX() {
        return leftX;
    }
    
    public double getLeftY() {
        return leftY;
    }

    public double getRightTriggerAxis() {
        return rightTriggerAxis;
    }

    public double getRightX() {
        return rightX;
    }

    public double getRightY() {
        return rightY;
    }

}
