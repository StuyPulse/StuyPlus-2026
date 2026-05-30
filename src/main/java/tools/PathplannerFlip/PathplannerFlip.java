package tools.PathplannerFlip;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.io.File;
import java.io.IOException;

public class PathplannerFlip {
    private static final double FIELD_HEIGHT = 8.0;
    private static final String PATH_DIR = "./src/main/deploy/pathplanner/paths/";

    public static void main(String[] args) throws IOException {
        if (args.length <= 0) {
            System.err.println("Missing path argument.");
            return;
        }

        final String pathName = args[0];
        final ObjectMapper mapper = new ObjectMapper();

        final File inputFile = new File(PATH_DIR + pathName + ".path");
        final ObjectNode root = (ObjectNode) mapper.readTree(inputFile);

        JsonNode waypoints = root.get("waypoints");
        if (waypoints != null) {
            for (JsonNode waypoint : waypoints) {
                flipWaypoint((ObjectNode) waypoint, "anchor");
                flipWaypoint((ObjectNode) waypoint, "prevControl");
                flipWaypoint((ObjectNode) waypoint, "nextControl");
            }
        }

        JsonNode pointTowardsZones = root.get("pointTowardsZones");
        if (pointTowardsZones != null) {
            for (JsonNode zone : pointTowardsZones) {
                flipWaypoint((ObjectNode) zone, "fieldPosition");
            }
        }

        mapper.writerWithDefaultPrettyPrinter().writeValue(new File(PATH_DIR + pathName + " Flipped.path"), root);
        System.out.println("Saved to " + pathName + " Flipped.path");
    }

    private static void flipWaypoint(ObjectNode parent, String fieldName) {
        JsonNode node = parent.get(fieldName);
        if (node == null || node.isNull())
            return;

        ObjectNode point = (ObjectNode) node;
        double flippedY = FIELD_HEIGHT - point.get("y").asDouble();
        point.put("y", flippedY);
    }
}