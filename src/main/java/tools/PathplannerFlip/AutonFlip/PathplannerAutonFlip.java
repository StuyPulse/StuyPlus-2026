package tools.PathplannerFlip.AutonFlip;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import tools.ToolClasses.VarArgumentedTool;
import tools.PathplannerFlip.PathFlip.PathplannerPathFlip;

import static tools.util.AnsiColors.*;

/**
 * <h2>Pathplanner Auton Flipper</h2>
 * <p>Flips autons of your choice across the y coordinate axis</p>
 * <p>This class extends {@link VarArgumentedTool} and takes in a list of auton names as arguments.
 * It reads each .auto file, flips the paths used in the auton, and saves a new .auto file with " Flipped" appended to the original name.</p>
 */
public class PathplannerAutonFlip extends VarArgumentedTool<String> {
    protected static final String AUTON_DIR = "./src/main/deploy/pathplanner/autos/";

    public PathplannerAutonFlip() {
        super(String.class, "The autons to flip over the y axis.");
    }

    public static boolean fileValid(File file) {
        return file.exists() && file.isFile() && file.canRead();
    }

    @Override
    protected void execute(List<String> arguments) {
        if (arguments.size() <= 0) {
            throw new IllegalArgumentException("Missing path argument.");
        }

        for (String autonName : arguments) {
            final File autonFile = new File(AUTON_DIR + autonName + ".auto");
            if (!PathplannerPathFlip.fileValid(autonFile)) {
                logPath(FLIP_STATUS.INVALID, autonName);
                continue;
            }

            final ObjectNode autonObj = ((Supplier<ObjectNode>) () -> {
                try {
                    return (ObjectNode) PathplannerPathFlip.mapper.readTree(autonFile);
                } catch (IOException e) {
                    e.printStackTrace();
                    return null;
                }
            }).get();
            if (autonObj == null) continue;

            ArrayNode commandArray = (ArrayNode) autonObj
                .path("command")
                .path("data")
                .path("commands");

            String[] pathNames = commandArray.valueStream()
                .filter(command -> command.path("type").asText().equals("path"))
                .map(command -> command.path("data").path("pathName").asText())
                .toArray(String[]::new);

            if (pathNames.length <= 0) {
                logPath(FLIP_STATUS.NO_PATHS, autonName);
                continue;
            }

            try {
                writeAuton(autonObj, autonName, PathplannerPathFlip.flipPaths(pathNames));
            } catch (IOException e) {
                e.printStackTrace();
            }
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
        logPath(FLIP_STATUS.FLIPPED, name);
    }

        private enum FLIP_STATUS {
        FLIPPED,
        INVALID,
        NO_PATHS
    }

    public static void logPath(FLIP_STATUS status, String pathName) {
        String coloredPathWithQuotes = YELLOW + "'" + pathName + "'" + RESET;
        String coloredPathWithoutQuotes = YELLOW + pathName + RESET;

        String message = switch (status) {
            case FLIPPED -> GREEN + "Saved to " + YELLOW + "'" + coloredPathWithoutQuotes + YELLOW + " Flipped.auto'" + GREEN + " successfully!" + RESET;
            case INVALID -> coloredPathWithQuotes + RED + " is an invalid auton." + RESET;
            case NO_PATHS -> coloredPathWithQuotes + RED + " has no path commands." + RESET;
        };

        System.out.println(message);
    }
}
