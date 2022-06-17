// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.net.URL;
import java.nio.charset.Charset;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/** Add your docs here. */
public class BreakerJsonUtil {
    public static JsonNode readJsonFromURL(String link) throws IOException {
        InputStream input = new URL(link).openStream();
        try {
            BufferedReader reader = new BufferedReader(new InputStreamReader(input, Charset.forName("UTF-8")));
            ObjectMapper mapper = new ObjectMapper();
            return mapper.readTree(reader);
        } catch (Exception e) {
            return null;
        } finally {
            input.close();
        }
    }
        
    private static String readString(Reader re) throws IOException {     // class Declaration
        StringBuilder str = new StringBuilder();     // To Store Url Data In String.
        int temp;
        do {
    
          temp = re.read();       //reading Charcter By Chracter.
          str.append((char) temp);
    
        } while (temp != -1);        
        //  re.read() return -1 when there is end of buffer , data or end of file. 
    
        return str.toString();
    
    }
}
