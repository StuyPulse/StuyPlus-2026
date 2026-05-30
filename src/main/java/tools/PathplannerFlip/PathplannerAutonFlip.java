package tools.PathplannerFlip;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

/**
 *
 *
 * <h2>Flip autons of your choice across the y coordinate axis</h2>
 *
 * <p>
 * To use, run:
 *
 * <pre>
 * ./gradlew runPathplannerAutonFlip -Pargs="[target autons]"
 * </pre>
 * 
 * Split each auton you want to flip with ", "
 * 
 * <p>
 * <b>Example:</b>
 *
 * <pre>
 * ./gradlew runPathplannerAutonFlip -Pargs="RB 1 Sweep Angled, Front Hub Shoot Preloads"
 * </pre>
 *
 * <p>
 * <b>Arguments:</b>
 *
 * <ul>
 * <li><b>[target autons]</b>: The autons to flip over the y axis.
 * </ul>
 */
public class PathplannerAutonFlip {
    protected static final String AUTON_DIR = "./src/main/deploy/pathplanner/autos/";

    public static void main(String[] args) throws IOException {
        if (args.length <= 0) {
            System.err.println("Missing path argument.");
            return;
        }
        flipAutons(args[0].split(", "));
    }

    public static void flipAutons(String... autons) throws IOException {
        for (String autonName : autons) {
            final File autonFile = new File(AUTON_DIR + autonName + ".auto");
            if (!PathplannerPathFlip.fileValid(autonFile)) {
                System.err.println(autonName + " - is NOT a valid auton </3");
                continue;
            }

            final ObjectNode autonObj = (ObjectNode) PathplannerPathFlip.mapper.readTree(autonFile);

            ArrayNode commandArray = (ArrayNode) autonObj
                .path("command")
                .path("data")
                .path("commands");

            String[] pathNames = commandArray.valueStream()
                .filter(command -> command.path("type").asText().equals("path"))
                .map(command -> command.path("data").path("pathName").asText())
                .toArray(String[]::new);

            if (pathNames.length <= 0) {
                System.err.println("The auton " + autonName + " has no path commands");
                continue;
            }

            writeAuton(autonObj, autonName, PathplannerPathFlip.flipPaths(pathNames));
        }
    }

    public static final ObjectNode blankPathObject = PathplannerPathFlip.mapper.createObjectNode()
        .put("type", "path")
        .set("data", PathplannerPathFlip.mapper.createObjectNode());

    public static void writeAuton(ObjectNode autonObj, String name, String[] paths) throws IOException {
        ObjectNode dataNode = (ObjectNode) autonObj
            .path("command")
            .path("data");
        
        ArrayNode flippedPaths = PathplannerPathFlip.mapper.createArrayNode();
        Arrays.stream(paths)
            .map(pathName -> {
                ObjectNode pathObj = (ObjectNode) blankPathObject.deepCopy().path("data");
                pathObj.put("pathName", pathName);
                return (ObjectNode) blankPathObject.deepCopy().set("data", pathObj);
            })
            .forEach(flippedPaths::add);
        dataNode.set("commands", flippedPaths);

        PathplannerPathFlip.mapper
            .writerWithDefaultPrettyPrinter()
            .writeValue(new File(AUTON_DIR + name + " Flipped.auto"), autonObj);
        System.out.println("Saved to " + name + " Flipped.auto");
    }
}
