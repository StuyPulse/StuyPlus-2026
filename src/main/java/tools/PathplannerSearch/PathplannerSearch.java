/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package tools.PathplannerSearch;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;

import java.util.ArrayList;
import java.util.List;

import java.util.regex.Pattern;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * <h2>Utility class that gives search features that Pathplanner lacks</h2>
 * <p>
 * This class is meant to help search for where paths and linked variables are
 * used so you can clean up unused ones.
 */
public final class PathplannerSearch {
    public enum SearchType {
        LINKED_WAYPOINT,
        PATH
    }

    public static void search(String searchTerm, SearchType searchType) throws IOException {
        switch (searchType) {
            case LINKED_WAYPOINT -> {
                final File[] files = new File("./src/main/deploy/pathplanner/paths/").listFiles();
                if (files == null)
                    break;

                final List<String> matches = new ArrayList<>();
                for (final File file : files) {
                    final String content = Files.readString(file.toPath()).toLowerCase();
                    if (content.isBlank())
                        continue;
                    if (content.contains("\"linkedname\"") && content.contains("\"" + searchTerm + "\"")) {
                        final String folderName = parseFolderKeyFromFileContent(content);
                        matches.add(folderName + "/" + file.getName());
                    }
                }

                logFiles(searchTerm, searchType, matches);
            }
            case PATH -> {
                final File[] files = new File("./src/main/deploy/pathplanner/autos/").listFiles();
                if (files == null)
                    break;

                final List<String> matches = new ArrayList<>();
                for (final File file : files) {
                    final String content = Files.readString(file.toPath()).toLowerCase();
                    if (!content.contains("\"sequential\""))
                        continue;
                    if (content.contains("\"path\"") && content.contains("\"pathname\"")
                            && content.contains("\"" + searchTerm + "\"")) {
                        
                        final String folderName = parseFolderKeyFromFileContent(content);
                        matches.add(folderName + "/" + file.getName());
                    }
                }

                logFiles(searchTerm, searchType, matches);
            }
            default -> {
                System.out.println("Specify a valid search type by editing the file.");
                System.exit(1);
            }
        }
    }

    private static String parseFolderKeyFromFileContent(String content) {
        final String folderName;
        Pattern regex = Pattern.compile("\"folder\"\\s*:\\s*\"([^\"]*)\"", Pattern.CASE_INSENSITIVE); // ima be honest i used ai for this regex 🤤
        Matcher match = regex.matcher(content);
        if (match.find()) {
            folderName = match.group(1);
        } else {
            folderName = "";
        }

        return folderName;
    }

    private static void logFiles(String searchTerm, SearchType searchType, List<String> matches) {
        final String RESET = "\u001B[0m";
        final String RED = "\u001B[31m";
        final String GREEN = "\u001B[32m";
        final String YELLOW = "\u001B[33m";

        String stem = switch (searchType) {
            case LINKED_WAYPOINT -> "Paths that use the linked waypoint '";
            case PATH -> "Autons that use the path '";
            default -> "";
        };

        System.out.println(stem + YELLOW + searchTerm + "'" + RESET + ":");
        if (matches.isEmpty()) {
            System.out.println(RED + "None were found." + RESET);
            return;
        }

        for (String match : matches) {
            System.out.println(GREEN + match + RESET);
        }
    }
}
