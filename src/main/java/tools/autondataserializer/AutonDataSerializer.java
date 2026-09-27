package tools.autondataserializer;

import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import com.fasterxml.jackson.core.exc.StreamWriteException;
import com.fasterxml.jackson.databind.DatabindException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.stuypulse.robot.util.PathUtil;
import com.stuypulse.robot.util.PathUtil.AutonConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonDataSerializer {

    public static void writeAutonData(final Path path, List<AutonConfig> autonConfigs)
            throws IOException, StreamWriteException, DatabindException {
        final var export = autonConfigs.stream()
                .map(auton -> {
                    final Command autonCommand = auton.auton().apply(PathUtil.loadPaths(auton.paths()));
                    return Map.of("name", auton.name(), "command_timings",
                            extractCommands((SequentialCommandGroup) autonCommand), "paths", auton.paths());
                }).toList();
        final ObjectMapper mapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
        mapper.writeValue(path.toFile(), export);
        System.out.println("Wrote " + export.size() + " auton configs to " + path.toString());
    }

    public static List<Double> extractCommands(final SequentialCommandGroup group) {
        try {
            final Field commandField = SequentialCommandGroup.class.getDeclaredField("m_commands");
            commandField.setAccessible(true);
            @SuppressWarnings("unchecked")
            final List<Command> commands = (List<Command>) commandField.get(group); // gg
            return commands.stream().map(command -> {
                final double completionTime = switch (command.getName()) {
                    case "FollowPathCommand" -> {
                        var pathCommand = (FollowPathCommand) command;
                        try {
                            final Field pathField = FollowPathCommand.class.getDeclaredField("trajectory");
                            pathField.setAccessible(true);
                            final PathPlannerTrajectory path = (PathPlannerTrajectory) pathField.get(pathCommand);
                            yield path.getTotalTimeSeconds();
                        } catch (NoSuchFieldException | SecurityException | IllegalAccessException e) {
                            e.printStackTrace();
                            yield 0.0;
                        }
                    }
                    default -> 0.0;
                };
                return completionTime;
                // return command.getName();
            }).toList();
        } catch (Exception e) {
            e.printStackTrace();
            return Collections.emptyList();
        }
    }
}
