// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.subsystem.cores.drivetrain.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.BreakerLib.util.BreakerArbitraryFeedforwardProvider;

/** Config class for {@link BreakerSwerveDrive}. */
public class BreakerSwerveDriveConfig {

    private double maxForwardVel;
    private double maxSidewaysVel;
    private double maxAngleVel;
    private int moduleNum;
    private double moduleAnglekP;
    private double moduleAnglekI;
    private double moduleAngleKd;
    private double moduleVelkP;
    private double moduleVelkI;
    private double moduleVelKd;
    private double moduleVelKf;
    private double driveMotorGearRatioToOne;
    private double wheelDiameter;
    private double slowModeLinearMultiplier;
    private double slowModeTurnMultiplier;
    private double moduleWheelSpeedDeadband;
    private double maxAttainableModuleWheelSpeed;
    private BreakerArbitraryFeedforwardProvider arbitraryFeedforwardProvider;

    private SwerveDriveKinematics kinematics;

    /**
     * The overall configuration for a Breaker swerve drivetrain holding all constants,
     * must be passed in.
     * 
     * @param maxForwardVel                  Max forward strafe velocity of drivetrain in
     *                                       m/s.
     * @param maxSidewaysVel                 Max horizontal strafe velocity of drivetrain in
     *                                       m/s.
     * @param maxAngVel                      Max angular velocity of the drivetrain
     *                                       in rad/s.
     * @param moduleAnglekP                  Swerve module angular kP (for PID).
     * @param moduleAnglekI                  Swerve module angular kI (for PID).
     * @param moduleAngleKd                  Swerve module angular kD (for PID).
     * @param moduleVelkP                    Swerve module drive kP (for PIDF).
     * @param moduleVelkI                    Swerve module drive kI (for PIDF).
     * @param moduleVelKd                    Swerve module drive kD (for PIDF).
     * @param moduleVelKf                    Swerve module drive kF (for PIDF).
     * @param arbitraryFeedforwardProvider
     * @param driveMotorGearRatioToOne       Gear ratio of swerve modules.
     * @param wheelDiameter                  The diameter of your module's wheels in inches.
     * @param wheelspeedDeadband             The min value (+/-) in m/s^2 that each modules wheel speed can be set too before being ingored
     * @param maxAttainableModuleWheelSpeed  The physical maximum speed (in m/s^2) your swerve modules are capable of achiving
     * @param wheelPositionsRelativeToCenter Position of each module relative to the robot's center as Translation2d ojects, must be
     *                                       in the same order as their respective modules are added to your BreakerSwerveDrive object.
     */
    public BreakerSwerveDriveConfig(double maxForwardVel, double maxSidewaysVel, double maxAngVel,
            double moduleAnglekP, double moduleAnglekI, double moduleAngleKd, double moduleVelkP,
            double moduleVelkI, double moduleVelKd, double moduleVelKf, double driveMotorGearRatioToOne,
            double wheelDiameter, double moduleWheelSpeedDeadband, double maxAttainableModuleWheelSpeed, BreakerArbitraryFeedforwardProvider arbitraryFeedforwardProvider,
            Translation2d... wheelPositionsRelativeToCenter) {

        this.maxForwardVel = maxForwardVel;
        this.maxSidewaysVel = maxSidewaysVel;
        this.maxAngleVel = maxAngVel;
        this.moduleAngleKd = moduleAngleKd;
        this.moduleAnglekI = moduleAnglekI;
        this.moduleAnglekP = moduleAnglekP;
        this.moduleVelKd = moduleVelKd;
        this.moduleVelkI = moduleVelkI;
        this.moduleVelkP = moduleVelkP;
        this.wheelDiameter = wheelDiameter;
        this.driveMotorGearRatioToOne = driveMotorGearRatioToOne;
        this.moduleVelKf = moduleVelKf;
        this.arbitraryFeedforwardProvider = arbitraryFeedforwardProvider;
        this.maxAttainableModuleWheelSpeed = maxAttainableModuleWheelSpeed;
        this.moduleWheelSpeedDeadband = moduleWheelSpeedDeadband;
        slowModeLinearMultiplier = 1;
        slowModeTurnMultiplier = 1;

        moduleNum = wheelPositionsRelativeToCenter.length;
        kinematics = new SwerveDriveKinematics(wheelPositionsRelativeToCenter);
    }

    /**
     * Sets slow mode multipliers.
     * 
     * @param linearMulitplier Slow mode multiplier for drive motors.
     * @param turnMultiplier Slow mode multiplier for turn motors.
     */
    public void setSlowModeMultipliers(double linearMulitplier, double turnMultiplier) {
        slowModeLinearMultiplier = linearMulitplier;
        slowModeTurnMultiplier = turnMultiplier;
    }

    /** @return Kinematics of swerve drivetrain. */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /** @return Max forward velocity of swerve drive in m/s. Usually the same as max sideways velocity. */
    public double getMaxForwardVel() {
        return maxForwardVel;
    }

    /** @return Max sideways velocity of swerve drive in m/s. Usually the same as max forward velocity. */
    public double getMaxSidewaysVel() {
        return maxSidewaysVel;
    }

    /** @return Max angular velocity of swerve drive in rad/s.*/
    public double getMaxAngleVel() {
        return maxAngleVel;
    }

    /** @return Number of swerve modules on swerve drivetrain. */
    public int getNumberOfModules() {
        return moduleNum;
    }

    /** @return kP value from velocity PIDF. */
    public double getModuleVelkP() {
        return moduleVelkP;
    }

    /** @return kI value from velocity PIDF. */
    public double getModuleVelkI() {
        return moduleVelkI;
    }

    /** @return kD value from velocity PIDF. */
    public double getModuleVelKd() {
        return moduleVelKd;
    }

    /** @return kF value from velocity PIDF. */
    public double getModuleVelKf() {
        return moduleVelKf;
    }

    /** @return kP value from angular PID. */
    public double getModuleAnglekP() {
        return moduleAnglekP;
    }

    /** @return kI value from angular PID. */
    public double getModuleAnglekI() {
        return moduleAnglekI;
    }

    /** @return kD value from angular PID. */
    public double getModuleAngleKd() {
        return moduleAngleKd;
    }

    /** @return Gear ratio of drive motors. */
    public double getDriveMotorGearRatioToOne() {
        return driveMotorGearRatioToOne;
    }

    /** @return Diameter of wheels in inches. */
    public double getWheelDiameter() {
        return wheelDiameter;
    }

    /** @return Number of swerve modules. */
    public int getNumberOfSwerveModules() {
        return moduleNum;
    }

    /** @return Slow mode multiplier on drive motors. */
    public double getSlowModeLinearMultiplier() {
        return slowModeLinearMultiplier;
    }

    /** @return Slow mode multiplier on turn motors. */
    public double getSlowModeTurnMultiplier() {
        return slowModeTurnMultiplier;
    }

    /** @return Arbitrary feedforward provider for swerve drive. */
    public BreakerArbitraryFeedforwardProvider getArbitraryFeedforwardProvider() {
        return arbitraryFeedforwardProvider;
    }

    public double getMaxAttainableModuleWheelSpeed() {
        return maxAttainableModuleWheelSpeed;
    }

    public double getModuleWheelSpeedDeadband() {
        return moduleWheelSpeedDeadband;
    }
}
