// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.music;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;

/** Class containing file locations of BreakerLib's bundled music and sounds */
public class BreakerSounds {
    private static final Path deployDir = Filesystem.getDeployDirectory().toPath();
    public static final String GeneralAlarmSound = deployDir.resolve("Breaker_General_Alarm.chrp").toString();
    public static final String startupSound = deployDir.resolve("Breaker_Startup.chrp").toString();
    public static final String enableSound = deployDir.resolve("Breaker_Enable.chrp").toString();
    public static final String disableSound = deployDir.resolve("Breaker_Disable.chrp").toString();
    public static final String slowAlarmSound = deployDir.resolve("Breaker_Alarm_Slow.chrp").toString();
    public static final String rushE = deployDir.resolve("Breaker_Rush_E.chrp").toString();
    public static final String megalovania = deployDir.resolve("Breaker_Megalovania.chrp").toString();
    public static final String turretOpera = deployDir.resolve("Turret Opera.chrp").toString();
}
