
/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.path.PathPlannerPath;
import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
// import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.function.Function;

public class PathUtil {
    public static class AutonConfig {
    
        private final String name;
        private final Function<PathPlannerPath[], Command> auton;
        private final String[] paths;

        public AutonConfig(String name, Function<PathPlannerPath[], Command> auton, String... paths) {
            this.name = name;
            this.auton = auton;
            this.paths = paths;

            for (String path : paths) {
                try {
                    PathPlannerPath.fromPathFile(path);
                } catch (Exception e) {
                    DriverStation.reportError("Path \"" + path + "\" not found. Did you mean \"" + PathUtil.findClosestMatch(PathUtil.getPathFileNames(), path) + "\"?", false);
                }
            }
        }
        
        public AutonConfig register(SendableChooser<Command> chooser) {
            chooser.addOption(name, auton.apply(loadPaths(paths)));
            return this;
        }
                
        public AutonConfig registerDefault(SendableChooser<Command> chooser) {
            chooser.setDefaultOption(name, auton.apply(loadPaths(paths)));
            return this;
        }
    }
    
    /*** PATH LOADING ***/

    public static PathPlannerPath[] loadPaths(String... names) {
        PathPlannerPath[] output = new PathPlannerPath[names.length];
        for (int i = 0; i < names.length; i++) {
            output[i] = load(names[i]);
        }
        return output;
    }

    public static PathPlannerPath load(String name) {
        try {
        return PathPlannerPath.fromPathFile(name);

        } catch (Exception e) {

            DriverStation.reportError("Path not found.", false);
            return null;
        }
    }

    /*** PATH FILENAME CORRECTION ***/

    public static List<String> getPathFileNames() {
        //  ../../../../../deploy/pathplanner/paths

        Path path = Paths.get("").toAbsolutePath().resolve("src/main/deploy/pathplanner/paths");
        ArrayList<String> fileList = new ArrayList<String>();
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(path, "*.path")) {
            for (Path file : stream) {
                fileList.add(file.getFileName().toString().replaceFirst(".path",""));
            }
        } catch (IOException error) {
            DriverStation.reportError(error.getMessage(), false);
        }
        Collections.sort(fileList);
        return fileList;
    }

    public static String findClosestMatch(List<String> paths, String input) {
        double closestValue = 10.0;
        String matching = "";

        for (String fileName : paths){
            HashMap<Character, Integer> fileChars = countChars(fileName.toCharArray());
            HashMap<Character, Integer> inputChars = countChars(input.toCharArray());

            double proximity = compareNameProximity(fileChars, inputChars);
            closestValue = Math.min(proximity, closestValue);

            if (proximity == closestValue) {
                matching = fileName;
            }
        }

        return matching;
    }
    
    public static HashMap<Character, Integer> countChars(char[] chars) {
        HashMap<Character, Integer> letterMap = new HashMap<>();
        for (char i = 'a'; i <= 'z'; i++) letterMap.put(i, 0);
        for (char i = 'a'; i <= 'z'; i++) letterMap.put(i, 0);
        letterMap.put('(', 0);
        letterMap.put(' ', 0);
        letterMap.put(')', 0);
        for (char letter : chars) {
            if (letterMap.containsKey(letter)) {
                letterMap.put(letter, letterMap.get(letter));
            } else {
                letterMap.put(letter, 1);
            }
        }
        return letterMap;
    }

    public static double compareNameProximity(HashMap<Character, Integer> list1, HashMap<Character, Integer> list2) {
        double proximity = 0.0;
        int list1sum = 0, list2sum = 0;
        for (char key : list1.keySet()) {
            if (!list2.containsKey(key)) {
                proximity += 0.1;
                continue;
            }
            proximity += 0.05 * Math.abs(list1.get(key) - list2.get(key));
        }
        for (char key : list2.keySet()) {
            if (!list1.containsKey(key)) {
                proximity += 0.1;
                continue;
            }
            proximity += 0.05 * Math.abs(list1.get(key) - list2.get(key));
        }
        for (int count : list1.values()) list1sum += count;
        for (int count : list2.values()) list2sum += count;
        proximity += 0.4 * Math.abs(list2sum - list1sum);
        return proximity;
    }
}