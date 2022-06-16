// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.selftest;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.fasterxml.jackson.databind.JsonNode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerGenericDevice;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerFalconOrchestra;
import frc.robot.BreakerLib.devices.cosmetic.music.BreakerSounds;
import frc.robot.BreakerLib.util.BreakerJsonUtil;
import frc.robot.BreakerLib.util.BreakerLog;

public class SelfTest extends SubsystemBase {
  /** Creates a new SelfTest. */
  private int cycleCount;
  private static String lastSystemCheck;
  private static List<BreakerSelfTestable> devices = new ArrayList<BreakerSelfTestable>();
  private static int cyclesbetweenPerSelfCecks = 250;
  private static boolean lastCheckPassed = true;
  private static BreakerFalconOrchestra orchestra;
  private static boolean usesOrchestra = false;
  private static String robotHostAddressDNS;
  private static List<Integer> retrivedDevicesCAN = new ArrayList<>();
  private static List<Integer> independentlyRegesteredDevicesCAN = new ArrayList<>();
  private static List<Integer> independentlyRegesteredMissingIDs = new ArrayList<>();
  private static boolean autoRegesterDevices = true;
  public SelfTest(double secondsBetweenPeriodicSelfChecks, String robotHostAddressDNS) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.usesOrchestra = false;
    SelfTest.robotHostAddressDNS = robotHostAddressDNS;
    SelfTest.autoRegesterDevices = true;
  }

  public SelfTest(double secondsBetweenPeriodicSelfChecks, String robotHostAddressDNS, boolean autoRegesterDevices) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.usesOrchestra = false;
    SelfTest.robotHostAddressDNS = robotHostAddressDNS;
    SelfTest.autoRegesterDevices = autoRegesterDevices;
  }

  public SelfTest(double secondsBetweenPeriodicSelfChecks,  String robotHostAddressDNS, BreakerFalconOrchestra orchestra) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.orchestra = orchestra;
    SelfTest.usesOrchestra = true;
    SelfTest.robotHostAddressDNS = robotHostAddressDNS;
  }

  public SelfTest(double secondsBetweenPeriodicSelfChecks,  String robotHostAddressDNS, BreakerFalconOrchestra orchestra, boolean autoRegesterDevices) {
    SelfTest.cyclesbetweenPerSelfCecks = (int) (secondsBetweenPeriodicSelfChecks * 50);
    SelfTest.orchestra = orchestra;
    SelfTest.usesOrchestra = true;
    SelfTest.robotHostAddressDNS = robotHostAddressDNS;
    SelfTest.autoRegesterDevices = autoRegesterDevices;
  }

  private static void retriveDeviceListCAN() {
    try {    
      JsonNode response = BreakerJsonUtil.readJsonFromURL("http://"+ robotHostAddressDNS +":1250/?action=getdevices").get("DeviceArray");
      Iterator<JsonNode> iter = response.elements();
      retrivedDevicesCAN.clear();
      while (iter.hasNext()) {
        JsonNode divNode = iter.next();
        retrivedDevicesCAN.add(divNode.get("uniqID").asInt());
      }
    } catch (Exception e) {
      BreakerLog.logError(e);
    }
    
  }

  public static void autoRegesterDevice(BreakerSelfTestable device) {
    if (autoRegesterDevices) {
      devices.add(device);
    }
  }

  public static void autoRegesterDevices(BreakerSelfTestable... devices) {
    if (autoRegesterDevices) {
      addDevices(devices);
    }
  }


  private static void runAlarm() {
    if (usesOrchestra) {
      orchestra.startLoopedSong(BreakerSounds.GeneralAlarmSound);
    }
  }

  public static void addDevice(BreakerSelfTestable device) {
    devices.add(device);
  }

  public static void addDevices(BreakerSelfTestable... devicesToAdd) {
    for (BreakerSelfTestable div: devicesToAdd) {
      devices.add(div);
    }
  }

  /** for CAN devices like moters that are not allredy automaticly tested by a BreakerLib class */
  public static void regesterGenericCANDevice(int deviceID) {
    independentlyRegesteredDevicesCAN.add(deviceID);
  }

  /** for CAN devices like moters that are not allredy automaticly tested by a BreakerLib class */
  public static void regesterGenericCANDevices(int... deviceIDs) {
    for (int id: deviceIDs) {
      independentlyRegesteredDevicesCAN.add(id);
    }
  }

  public static String getLastSelfCheck() {
    return lastSystemCheck;
  }

  public static boolean getLastSelfCheckPassed() {
    return lastCheckPassed;
  }

  public static boolean checkIsMissingCanID(int deviceID) {
    return retrivedDevicesCAN.contains(deviceID);
  }

  public static boolean getAutoRegesterDevicesIsEnabled() {
    return autoRegesterDevices;
  }

  public static void runSelfCheck() {
    StringBuilder work = new StringBuilder("\n RUNNING SELF CHECK: \n");
    List<BreakerSelfTestable> faultDevices = new ArrayList<BreakerSelfTestable>();
    for (BreakerSelfTestable device: devices) {
      device.runSelfTest();
      if (device.hasFault()) {
        faultDevices.add(device);
      }
    }
    if (!independentlyRegesteredMissingIDs.isEmpty()) {
      for (int id: independentlyRegesteredDevicesCAN) {
        if (checkIsMissingCanID(id)) {
          independentlyRegesteredMissingIDs.add(id);
        }
      }
    }
    if (!faultDevices.isEmpty() || !independentlyRegesteredMissingIDs.isEmpty()) {
      work.append(" SELF CHECK FAILED - FAULTS FOUND: \n");
      lastCheckPassed = false;
      for (BreakerSelfTestable faultDiv: faultDevices) {
        work.append(" | " + faultDiv.getDeviceName() + "-" + faultDiv.getFaults() + " | ");
      }
      for (int id: independentlyRegesteredMissingIDs) {
        work.append(" | CAN device (ID: " + id + ") - NOT_FOUND_ON_BUS | ");
      }
      runAlarm();
    } else {
      work.append(" SELF CHECK PASSED ");
      lastCheckPassed = true;
    }
    lastSystemCheck = work.toString();
    BreakerLog.log(lastSystemCheck);
  }

  @Override
  public void periodic() {
    if (cycleCount ++ % cyclesbetweenPerSelfCecks == 0) {
      retriveDeviceListCAN();
      runSelfCheck();
    }
  }
}
