package tools.PathplannerSearch;

import java.io.IOException;
import tools.PathplannerSearch.PathplannerSearch.SearchType;

/**
 * <h2>Main class for PathplannerSearch</h2>
 *
 * <p>To use, run:
 * <pre>
 * ./gradlew runPathplannerSearch -Pargs="[search term] [search type]"
 * </pre>
 *
 * <p><b>Example:</b>
 * <pre>
 * ./gradlew runPathplannerSearch -Pargs="disruptAUton path"
 * </pre>
 *
 * <p><b>Arguments:</b>
 * <ul>
 *   <li><b>[search term]</b>: The term to search for in the Pathplanner files.</li>
 *   <li><b>[search type]</b>: The type of search to perform:
 *     <ul>
 *       <li>{@code linked_waypoint}</li>
 *       <li>{@code path}</li>
 *     </ul>
 *   </li>
 * </ul>
 */
public class Main {
    private enum Arguments {
        SEARCH_TERM,
        SEARCH_TYPE;
    }

    public static void main(String[] args) throws IOException {
        String searchTerm = 
            args[Arguments.SEARCH_TERM.ordinal()] != null ? 
                args[Arguments.SEARCH_TERM.ordinal()].toLowerCase() 
                : "";
        SearchType searchType = 
            args[Arguments.SEARCH_TYPE.ordinal()] != null ? 
                SearchType.valueOf(args[Arguments.SEARCH_TYPE.ordinal()].toUpperCase()) 
                : SearchType.PATH;

        PathplannerSearch.search(searchTerm, searchType);
    }
}