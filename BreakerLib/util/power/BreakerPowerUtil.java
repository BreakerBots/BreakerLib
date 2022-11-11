// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.power;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

/** Add your docs here. */
public class BreakerPowerUtil {
    public static List<BreakerPowerChannel> getNewPowerChannelList(ModuleType powerDistributionModuleType) {
        List<BreakerPowerChannel> channelList = new ArrayList<>();
        for (int i = 0; i <= (powerDistributionModuleType == ModuleType.kCTRE ? 23 : 15); i++) {
            channelList.add(new BreakerPowerChannel(i));
        }
        return channelList;
    }
}
