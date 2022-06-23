// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.position.odometry.BreakerGenericOdometer;

/** Add your docs here. */
public class BreakerFieldWidget extends SubsystemBase{
    private static Field2d field = new Field2d();
    private static BreakerGenericOdometer odometer;
    public BreakerFieldWidget(BreakerGenericOdometer odometer) {
        BreakerFieldWidget.odometer = odometer;
        BreakerDashboard.getMainTab().add(field);
    }

    public static FieldObject2d getRobotObject() {
        return field.getRobotObject();
    }

    public static FieldObject2d getFieldObject(String name) {
        return field.getObject(name);
    }

    public static Field2d getBaseField() {
        return field;
    }

    @Override
    public void periodic() {
        field.setRobotPose(odometer.getOdometryPoseMeters());
    }
}
