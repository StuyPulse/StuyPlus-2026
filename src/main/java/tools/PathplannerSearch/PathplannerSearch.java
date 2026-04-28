package tools.PathplannerSearch;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

/*
 * <h2>Utility class that gives search features that Pathplanner lacks</h2>
 * <p>This class is meant to help search for where paths and linked variables are used so you can clean up unused ones.
 */
public final class PathplannerSearch {
    public enum SearchType {
        LINKED_WAYPOINT,
        PATH
    }

    public static void search(String searchTerm, SearchType searchType) throws IOException {
        final String SEARCH_TERM = searchTerm.toLowerCase();
        final SearchType SEARCH_TYPE = searchType;

        switch (SEARCH_TYPE) {
            case LINKED_WAYPOINT -> {
                final File[] files = new File("./src/main/deploy/pathplanner/paths/").listFiles();
                if (files == null) break;

                final List<String> matches = new ArrayList<>();
                for (final File file : files) {
                    final String content = Files.readString(file.toPath()).toLowerCase();
                    if (content.isBlank()) continue;
                    if (content.contains("\"linkedname\"") && content.contains("\"" + SEARCH_TERM + "\"")) {
                        matches.add(file.getName());
                    }
                }
                System.out.println("Paths that use the linked waypoint '" + SEARCH_TERM + "':\n" + (matches.isEmpty() ? "none" : String.join(", ", matches)));
            }
            case PATH -> {
                final File[] files = new File("./src/main/deploy/pathplanner/autos/").listFiles();
                if (files == null) break;

                final List<String> matches = new ArrayList<>();
                for (final File file : files) {
                    final String content = Files.readString(file.toPath()).toLowerCase();
                    if (!content.contains("\"sequential\"")) continue;
                    if (content.contains("\"path\"") && content.contains("\"pathname\"") && content.contains("\"" + SEARCH_TERM + "\"")) {
                        matches.add(file.getName());
                    }
                }
                System.out.println("Autons that use the path '" + SEARCH_TERM + "':\n" + (matches.isEmpty() ? "none" : String.join(", ", matches)));
            }
            default -> {
                System.out.println("Specify a valid search type by editing the file.");
                System.exit(1);
            }
        }
    }
}
