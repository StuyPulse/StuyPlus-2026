/************************ PROJECT TRIBECBOT ***********************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.lib.BLine.Path;
import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import com.stuypulse.robot.commands.auton.bline.BLineAuton;

public class BlineUtil {

    public static final java.io.File PATHS_DIR = Paths.get("").toAbsolutePath().resolve("src/main/deploy/bline/autos").toFile();

    public static class BLineConfig {

        private final String name;
        private final Function<Path[], BLineAuton> auton;
        private final String[] paths;
        private final Optional<Double> waitTimeOne;
        private final Optional<Double> waitTimeTwo;

        public BLineConfig(
                String name, 
                Function<Path[], 
                BLineAuton> auton, 
                double waitTimeOne, 
                double waitTimeTwo, 
                String... paths) {
            this.name = name;
            this.auton = auton;
            this.paths = paths;
            this.waitTimeOne = Optional.of(waitTimeOne);
            this.waitTimeTwo = Optional.of(waitTimeTwo);

            for (String path : paths) {
                try {
                    new Path(PATHS_DIR, path);
                } catch (Exception e) {
                    DriverStation.reportError(
                        "BLine path \"" + path + "\" not found. Did you mean \""
                            + BlineUtil.findClosestMatch(BlineUtil.getPathFileNames(), path) + "\"?",
                        false);
                }
            }
        }

        public BLineConfig(String name, Function<Path[], BLineAuton> auton, String... paths) {
            this(name, auton, 0.0, 0.0, paths);
        }

        private BLineAuton buildBLineAuton() {
            return auton.apply(loadPaths(paths));   // pass names straight through, no pre-loading
        }

        public BLineConfig register(SendableChooser<BLineAuton> chooser) {
            chooser.addOption(name, buildBLineAuton());
            return this;
        }

        public BLineConfig registerDefault(SendableChooser<BLineAuton> chooser) {
            chooser.setDefaultOption(name, buildBLineAuton());
            return this;
        }
    }

    /*** PATH LOADING ***/

    public static Path[] loadPaths(String... names) {
        Path[] output = new Path[names.length];
        for (int i = 0; i < names.length; i++) {
            output[i] = load(names[i]);
        }
        return output;
    }

    public static Path load(String name) {
        try {
            return new Path(PATHS_DIR, name);
        } catch (Exception e) {
            DriverStation.reportError("BLine path \"" + name + "\" not found.", false);
            return null;
        }
    }

    /*** PATH FILENAME CORRECTION ***/

    public static List<String> getPathFileNames() {
        // ../../../../../deploy/bline/autos/paths
        java.nio.file.Path dir = Paths.get("").toAbsolutePath().resolve("src/main/deploy/bline/autos/paths");
        ArrayList<String> fileList = new ArrayList<String>();
        try (DirectoryStream<java.nio.file.Path> stream = Files.newDirectoryStream(dir, "*.json")) {
            for (java.nio.file.Path file : stream) {
                fileList.add(file.getFileName().toString().replaceFirst(".json", ""));
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

        for (String fileName : paths) {
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