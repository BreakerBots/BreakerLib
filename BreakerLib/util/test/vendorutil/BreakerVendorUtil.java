// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.vendorutil;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import frc.robot.BreakerLib.util.test.selftest.DeviceHealth;

/**
 * This class provides util methods for use across devices from multiple vendors
 * such as bit-field fault search methods and more.
 */
public class BreakerVendorUtil {

    /** Returns device health as {@link DeviceHealth} values. */
    public static DeviceHealth getDeviceFaultsAsHealth(long faultBitField,
            HashMap<Integer, DeviceHealth> fieldPlacesAndHealthEffects) {
        DeviceHealth health = DeviceHealth.NOMINAL;
        if (faultBitField != 0) {
            long fieldMask = 1; // masks all but selected bit
            for (int fieldPlace = 0; fieldPlace < fieldPlacesAndHealthEffects.size(); fieldPlace++) {
                if (((faultBitField & fieldMask) != 0) && fieldPlacesAndHealthEffects.containsKey(fieldPlace)) { // Checks
                                                                                                                 // for
                                                                                                                 // 1s
                                                                                                                 // in
                                                                                                                 // bitfield
                                                                                                                 // that
                                                                                                                 // signifies
                                                                                                                 // error
                    health = health != DeviceHealth.INOPERABLE ? fieldPlacesAndHealthEffects.get(fieldPlace) : health;
                }
                fieldMask <<= 1; // Scrolls to next bit.
            }
        }
        return health;
    }

    /** Gets device health and faults. */
    public static Pair<DeviceHealth, String> getDeviceHealthAndFaults(long faultBitField,
            HashMap<Integer, Pair<DeviceHealth, String>> fieldPlacesHealthEffectsAndFaultMessages) {
        StringBuilder work = new StringBuilder();
        DeviceHealth health = DeviceHealth.NOMINAL;
        if (faultBitField != 0) {
            long fieldMask = 1; // masks all but selected bit
            for (int fieldPlace = 0; fieldPlace < fieldPlacesHealthEffectsAndFaultMessages.size(); fieldPlace++) {
                if (((faultBitField & fieldMask) != 0)
                        && fieldPlacesHealthEffectsAndFaultMessages.containsKey(fieldPlace)) { // Checks for 1s in
                                                                                               // bitfield that
                                                                                               // signifies error
                    work.append(fieldPlacesHealthEffectsAndFaultMessages.get(fieldPlace).getSecond());
                    health = health != DeviceHealth.INOPERABLE
                            ? fieldPlacesHealthEffectsAndFaultMessages.get(fieldPlace).getFirst()
                            : health;
                }
                fieldMask <<= 1; // Scrolls to next bit.
            }
        }
        return new Pair<DeviceHealth, String>(health, work.toString());
    }
}
