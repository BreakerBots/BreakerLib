// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.differential;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGenericGyro;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;
import frc.robot.BreakerLib.util.test.vendorutil.BreakerREVUtil;

/** A {@link BreakerDiffDrive} instance with SparkMax driven Neo motors */
public class BreakerNeoDiffDrive extends BreakerDiffDrive {
    private CANSparkMax[] leftMotors, rightMotors;
    /** Creates a new Differential (tank drive) drivetrain instance.
     * 
     * @param leftMotors an array of the {@link CANSparkMax} motor controlers that controll the drive's left side
     * @param invertL a boolean the determens wheather or not to invert the left drive motor output and encoder readings
     * @param rightMotors an array of the {@link CANSparkMax} motor controlers that controll the drive's right side
     * @param invertR a boolean the determens wheather or not to invert the left drive motor output and encoder readings
     * @param gyro a {@link BreakerGenericGyro} yaw reading capable gyroscope or IMU
     * @param driveConfig A {@link BreakerDiffDriveConfig} representing the configerable values of this drivetrain's kinimatics and control values
     */
    public BreakerNeoDiffDrive(CANSparkMax[] leftMotors, boolean invertL, CANSparkMax[] rightMotors, boolean invertR, 
        BreakerGenericGyro gyro, BreakerDiffDriveConfig driveConfig) {
        super(leftMotors, () -> ((Double)(leftMotors[0].getEncoder().getPosition() / driveConfig.getEncoderTicks())), () -> ((Double)leftMotors[0].getEncoder().getVelocity()), invertL, rightMotors,  () -> ((Double)(leftMotors[0].getEncoder().getPosition() / driveConfig.getEncoderTicks())), () -> ((Double)leftMotors[0].getEncoder().getVelocity()), invertR,
                gyro, driveConfig);
    }

    @Override
    public void runSelfTest() {
        faultStr = null;
        health = DeviceHealth.NOMINAL;

        StringBuilder work = new StringBuilder();
        for (CANSparkMax motorL : leftMotors) {
            short faults = motorL.getFaults();
            if ((int) faults != 0) {
                health = DeviceHealth.FAULT;
                work.append(" MOTOR ID (" + motorL.getDeviceId() + ") FAULTS: ");
                work.append(BreakerREVUtil.getSparkMaxHealthAndFaults(faults).getSecond());
            }
        }
        for (CANSparkMax motorR : rightMotors) {
            short faults = motorR.getFaults();
            if ((int) faults != 0) {
                health = DeviceHealth.FAULT;
                work.append(" MOTOR ID (" + motorR.getDeviceId() + ") FAULTS: ");
                work.append(BreakerREVUtil.getSparkMaxHealthAndFaults(faults).getSecond());
            }
        }
        faultStr = work.toString();
    }

    @Override
    public void resetDriveEncoders() {
        leftMotors[0].getEncoder().setPosition(0);
        rightMotors[0].getEncoder().setPosition(0);
        
    }

    @Override
    public void setDrivetrainBrakeMode(boolean isEnabled) {
        BreakerREVUtil.setBrakeMode(isEnabled, leftMotors);
        BreakerREVUtil.setBrakeMode(isEnabled, rightMotors);
    }

}
