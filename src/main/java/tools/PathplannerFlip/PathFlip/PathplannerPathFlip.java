/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package tools.PathplannerFlip.PathFlip;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import tools.ToolClasses.VarArgumentedTool;

/**
 * <h2>Pathplanner Path Flipper</h2>
 * <p>Flips paths of your choice across the y coordinate axis</p>
 * <p>This class extends {@link VarArgumentedTool} and takes in a list of path names as arguments.
 * It reads each .path file, flips the coordinates, and saves a new .path file with " Flipped" appended to the original name.</p>
 */
public final class PathplannerPathFlip extends VarArgumentedTool<String> {
    public static final double FIELD_HEIGHT = 8.0;
    public static final String PATH_DIR = "./src/main/deploy/pathplanner/paths/";

    public static final ObjectMapper mapper = new ObjectMapper();

    public PathplannerPathFlip() {
        super(String.class, "The paths to flip over the y axis.");
    }

    @Override
    protected void execute(List<String> arguments) {
        if (arguments.isEmpty()) {
            System.err.println("Missing path argument.");
            return;
        }

        try {
            flipPaths(arguments.toArray(new String[0]));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static boolean fileValid(File file) {
        return file.exists() && file.isFile() && file.canRead();
    }

    public static String[] flipPaths(String ...paths) throws IOException {
        final ArrayList<String> flippedPaths = new ArrayList<>();
        for (String pathName : paths) {
            final File pathFile = new File(PATH_DIR + pathName + ".path");
            if (!fileValid(pathFile)) {
                logPath(FLIP_STATUS.INVALID, pathName);
                continue;
            }

            final ObjectNode pathObj = (ObjectNode) mapper.readTree(pathFile);

            JsonNode waypoints = pathObj.path("waypoints");
            if (waypoints.isMissingNode()) {
                logPath(FLIP_STATUS.MISSING_FIELDS, pathName);
                continue;
            }

            for (JsonNode waypoint : waypoints) {
                flipWaypoint((ObjectNode) waypoint, "anchor");
                flipWaypoint((ObjectNode) waypoint, "prevControl");
                flipWaypoint((ObjectNode) waypoint, "nextControl");
            }

            JsonNode rotationTargets = pathObj.get("rotationTargets");
            if (rotationTargets != null) {
                for (JsonNode target : rotationTargets) {
                    ObjectNode targetObj = (ObjectNode) target;
                    double flipped = -targetObj.path("rotationDegrees").asDouble();
                    targetObj.put("rotationDegrees", flipped);
                }
            }

            JsonNode goalEndState = pathObj.get("goalEndState");
            if (goalEndState != null) {
                double flipped = -goalEndState.path("rotation").asDouble();
                ((ObjectNode) goalEndState).put("rotation", flipped);
            }

            JsonNode idealStartingState = pathObj.get("idealStartingState");
            if (idealStartingState != null) {
                double flipped = -idealStartingState.path("rotation").asDouble();
                ((ObjectNode) idealStartingState).put("rotation", flipped);
            }

            JsonNode pointTowardsZones = pathObj.get("pointTowardsZones");
            if (pointTowardsZones != null) {
                for (JsonNode zone : pointTowardsZones) {
                    flipWaypoint((ObjectNode) zone, "fieldPosition");
                }
            }

            final File flippedFile = new File(PATH_DIR + pathName + " Flipped.path");
            mapper.writerWithDefaultPrettyPrinter().writeValue(flippedFile, pathObj);
            logPath(FLIP_STATUS.FLIPPED, pathName);
            flippedPaths.add(pathName + " Flipped");
        }
        return flippedPaths.toArray(new String[0]);
    }

    public static void flipWaypoint(ObjectNode parent, String fieldName) {
        JsonNode node = parent.get(fieldName);
        if (node == null || node.isNull())
            return;

        ObjectNode point = (ObjectNode) node;
        double flippedY = FIELD_HEIGHT - point.get("y").asDouble();
        point.put("y", flippedY);
    }

            private enum FLIP_STATUS {
        FLIPPED,
        INVALID,
        MISSING_FIELDS
    }

    public static void logPath(FLIP_STATUS status, String pathName) {
        final String RESET = "\u001B[0m";
        final String RED = "\u001B[31m";
        final String GREEN = "\u001B[32m";
        final String YELLOW = "\u001B[33m";

        String coloredPathWithQuotes = YELLOW + "'" + pathName + "'" + RESET;
        String coloredPathWithoutQuotes = YELLOW + pathName + RESET;

        String message = switch (status) {
            case FLIPPED -> GREEN + "Saved to " + YELLOW + "'" + coloredPathWithoutQuotes + YELLOW + " Flipped.path'" + GREEN + " successfully!" + RESET;
            case INVALID -> coloredPathWithQuotes + RED + " is an invalid path." + RESET;
            case MISSING_FIELDS -> coloredPathWithQuotes + RED + " is missing required fields." + RESET;
        };

        System.out.println(message);
    }
}
