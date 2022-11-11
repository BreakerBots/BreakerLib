// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.logging;

/** Add your docs here. */
public class BreakerTelemetry {
    private static boolean saveTelemetry = false;
    public static void start(boolean saveTelemetry) {
        BreakerTelemetry.saveTelemetry = saveTelemetry;
    }

    public static void logTelemetry(boolean telemIsEnabled, String providerName, String data) {
        if (telemIsEnabled) {
            String logStr = "TELEM (" + providerName + "):" + data;
            if (saveTelemetry) {
                BreakerLog.log(logStr);
            } else {
                System.out.println(logStr);
            }
        }
    }
}
