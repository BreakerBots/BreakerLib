// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robotmanager;

import frc.robot.BreakerLib.auto.trajectory.management.BreakerAutoPath;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;

/** Config class for the robot, used by {@link BreakerRobotManager}. */
public class BreakerRobotConfig {

    private double secondsBetweenSelfChecks;
    private boolean autologNetworkTables;
    private boolean autoRegisterDevices;
    private BreakerFalconOrchestra orchestra;
    private boolean usesOrchestra;
    private BreakerAutoPath[] autoPaths;
    private boolean usesPaths;
    private BreakerRobotStartConfig startConfig;

    /**
     * Robot config with Falcon orchestra and auto path. Devices are automatically
     * registered to SelfTest.
     * 
     * @param secondsBetweenSelfChecks Time between each self-check in seconds.
     * @param autoRegisterDevices      Whether or not to automatically register
     *                                 devices into SelfTest.
     * @param autologNetworkTables     Whether or not to log NetworkTables.
     * @param startConfig              Startup info to use.
     * @param orchestra                BreakerFalconOrchestra to use.
     * @param autoPaths                Auto paths to initialize.
     */
    public BreakerRobotConfig(double secondsBetweenSelfChecks, boolean autoRegisterDevices,
            boolean autologNetworkTables, BreakerRobotStartConfig startConfig, BreakerFalconOrchestra orchestra,
            BreakerAutoPath... autoPaths) {
        this.secondsBetweenSelfChecks = secondsBetweenSelfChecks;
        this.autologNetworkTables = autologNetworkTables;
        this.orchestra = orchestra;
        this.autoPaths = autoPaths;
        this.autoRegisterDevices = autoRegisterDevices;
        this.startConfig = startConfig;
        usesOrchestra = true;
        usesPaths = true;
    }

    /**
     * Robot config with Falcon orchestra and no auto paths. Devices are not
     * automatically registered to SelfTest.
     * 
     * @param secondsBetweenSelfChecks Time between each self-check in seconds.
     * @param autologNetworkTables     Whether or not to log NetworkTables.
     * @param startConfig              Startup info to use.
     * @param orchestra                BreakerFalconOrchestra to use.
     */
    public BreakerRobotConfig(double secondsBetweenSelfChecks, boolean autologNetworkTables,
            BreakerRobotStartConfig startConfig, BreakerFalconOrchestra orchestra) {
        this.secondsBetweenSelfChecks = secondsBetweenSelfChecks;
        this.autologNetworkTables = autologNetworkTables;
        this.orchestra = orchestra;
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = startConfig;
        usesOrchestra = true;
        usesPaths = false;
    }

    /**
     * Robot config with auto paths. Falcon orchestra will not be used, devices will
     * not be automatically registered to SelfTest.
     * 
     * @param secondsBetweenSelfChecks Time between each self-check in seconds.
     * @param autologNetworkTables     Whether or not to log NetworkTables.
     * @param startConfig              Startup info to use.
     * @param autoPaths                Auto paths to select from.
     */
    public BreakerRobotConfig(double secondsBetweenSelfChecks, boolean autologNetworkTables,
            BreakerRobotStartConfig startConfig, BreakerAutoPath... autoPaths) {
        this.secondsBetweenSelfChecks = secondsBetweenSelfChecks;
        this.autologNetworkTables = autologNetworkTables;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = autoPaths;
        this.startConfig = startConfig;
        usesOrchestra = false;
        usesPaths = true;
    }

    /**
     * Robot config without auto paths or Falcon orchestra support. Devices will
     * not be automatically registered to SelfTest.
     * 
     * @param secondsBetweenSelfChecks Time between each self-check in seconds.
     * @param autologNetworkTables     Whether or not to log NetworkTables.
     * @param startConfig              Startup info to use.
     */
    public BreakerRobotConfig(double secondsBetweenSelfChecks, boolean autologNetworkTables,
            BreakerRobotStartConfig startConfig) {
        this.secondsBetweenSelfChecks = secondsBetweenSelfChecks;
        this.autologNetworkTables = autologNetworkTables;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = startConfig;
        usesOrchestra = false;
        usesPaths = false;
    }

    /**
     * Robot config with auto paths and Falcon Orchestra support. Self-checks will be run every 5 seconds.
     * 
     * @param secondsBetweenSelfChecks Time between each self-check in seconds.
     * @param orchestra                BreakerFalconOrchestra to use.
     * @param autoPaths                Auto paths to initialize.
     */
    public BreakerRobotConfig(BreakerRobotStartConfig startConfig, BreakerFalconOrchestra orchestra,
            BreakerAutoPath... autoPaths) {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegisterDevices = true;
        this.orchestra = orchestra;
        this.autoPaths = autoPaths;
        this.startConfig = startConfig;
        usesOrchestra = true;
        usesPaths = true;
    }

    /**
     * Robot config with auto paths. Self-checks will be run every 5 seconds.
     * 
     * @param secondsBetweenSelfChecks Time between each self-check in seconds.
     * @param orchestra                BreakerFalconOrchestra to use.
     * @param autoPaths                Auto paths to initialize.
     */
    public BreakerRobotConfig(BreakerRobotStartConfig startConfig, BreakerFalconOrchestra orchestra) {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegisterDevices = true;
        this.orchestra = orchestra;
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = startConfig;
        usesOrchestra = true;
        usesPaths = false;
    }

    public BreakerRobotConfig(BreakerRobotStartConfig startConfig, BreakerAutoPath... autoPaths) {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegisterDevices = true;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = autoPaths;
        this.startConfig = startConfig;
        usesOrchestra = false;
        usesPaths = true;
    }

    /**
     * Default robot configuration with provided BreakerStartConfig. Pre-assigned settings are listed below:
     * <p>
     * - Seconds between self-checks = 5.
     * <p>
     * - Devices are automatically registered into SelfTest.
     * <p>
     * - NetworkTables aren't logged.
     * <p>
     * - Auto paths are not used.
     * <p>
     * - Falcon Orchestra is not used.
     * 
     * @param startConfig {@link BreakerRobotStartConfig} to use.
     */
    public BreakerRobotConfig(BreakerRobotStartConfig startConfig) {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegisterDevices = true;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = startConfig;
        usesOrchestra = false;
        usesPaths = false;
    }

    /**
     * Default robot configuration. Pre-assigned settings are listed below:
     * <p>
     * - Seconds between self-checks = 5.
     * <p>
     * - Devices are automatically registered into SelfTest.
     * <p>
     * - NetworkTables aren't logged.
     * <p>
     * - Auto paths are not used.
     * <p>
     * - Falcon Orchestra is not used.
     * <p>
     * - Default start config is used.
     */
    public BreakerRobotConfig() {
        this.secondsBetweenSelfChecks = 5;
        this.autologNetworkTables = false;
        this.autoRegisterDevices = true;
        this.orchestra = new BreakerFalconOrchestra();
        this.autoPaths = new BreakerAutoPath[0];
        this.startConfig = new BreakerRobotStartConfig();
        usesOrchestra = false;
        usesPaths = false;
    }

    public boolean getAutoRegisterDevices() {
        return autoRegisterDevices;
    }

    public BreakerAutoPath[] getAutoPaths() {
        return autoPaths;
    }

    public BreakerFalconOrchestra getOrchestra() {
        return orchestra;
    }

    public double getSecondsBetweenSelfChecks() {
        return secondsBetweenSelfChecks;
    }

    public boolean getAutologNetworkTables() {
        return autologNetworkTables;
    }

    public boolean UsesOrchestra() {
        return usesOrchestra;
    }

    public boolean UsesPaths() {
        return usesPaths;
    }

    public BreakerRobotStartConfig getStartConfig() {
        return startConfig;
    }

}
