package tools.PathplannerFlip;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

/**
 *
 *
 * <h2>Flip paths of your choice across the y coordinate axis</h2>
 *
 * <p>
 * To use, run:
 *
 * <pre>
 * ./gradlew runPathplannerPathFlip -Pargs="[target paths]"
 * </pre>
 * 
 * Split each paths you want to flip with ", "
 * 
 * <p>
 * <b>Example:</b>
 *
 * <pre>
 * ./gradlew runPathplannerPathFlip -Pargs="RB to Mid Neutral LF, Mid Neutral LF to RB Shoot"
 * </pre>
 *
 * <p>
 * <b>Arguments:</b>
 *
 * <ul>
 * <li><b>[target paths]</b>: The paths to flip over the y axis.
 * </ul>
 */
public class PathplannerPathFlip {
    public static final double FIELD_HEIGHT = 8.0;
    public static final String PATH_DIR = "./src/main/deploy/pathplanner/paths/";

    public static final ObjectMapper mapper = new ObjectMapper();
    public static void main(String[] args) throws IOException {
        if (args.length <= 0) {
            System.err.println("Missing path argument.");
            return;
        }
        flipPaths(args[0].split(", "));
    }

    public static boolean fileValid(File file) {
        return file.exists() && file.isFile() && file.canRead();
    }

    public static String[] flipPaths(String ...paths) throws IOException {
        final ArrayList<String> flippedPaths = new ArrayList<>();
        for (String pathName : paths) {
            final File pathFile = new File(PATH_DIR + pathName + ".path");
            if (!fileValid(pathFile)) {
                System.err.println(pathName + " - is NOT a valid path </3");
                continue;
            }

            final ObjectNode pathObj = (ObjectNode) mapper.readTree(pathFile);

            JsonNode waypoints = pathObj.path("waypoints");
            if (waypoints.isMissingNode()) {
                System.err.println("The path " + pathName + " is missing fields.");
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
            System.out.println("Saved to " + pathName + " Flipped.path");
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
}