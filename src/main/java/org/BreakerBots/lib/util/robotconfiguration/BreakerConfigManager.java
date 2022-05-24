// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.robotconfiguration;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.BreakerLib.util.BreakerLog;


/** Add your docs here. */
public class BreakerConfigManager {
    private static List<JsonNode> configs;
    public static void addConfig(String configFileName) {
        try {
            String configFilePath = Filesystem.getDeployDirectory() + "/" + configFileName;
            BufferedReader reader = Files.newBufferedReader(Paths.get(configFilePath));
            ObjectMapper mapper = new ObjectMapper();
            configs.add(mapper.readTree(reader)); 
        } catch (Exception e) {
            BreakerLog.logError("FAILED_TO_PARSE_CONFIG " + e + "\n" + "FILENAME: " + configFileName);
        }
    }

    public static JsonNode getConfig(String configType) throws Exception{
        for (JsonNode conf: configs) {
            if (conf.path("configType").asText() == configType) {
                return conf;
            }
        }
        throw new Exception("CONFIG_NOT_FOUND");
    }
}
