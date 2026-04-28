package tools.PathplannerSearch;

import java.io.IOException;
import tools.PathplannerSearch.PathplannerSearch.SearchType;

/*
 * <h3>Utility class that gives search features that Pathplanner lacks</h3>
 * <p>This class is meant to help search for where paths and linked variables are used so you can clean up unused ones.
 * <br>Use by first changing the SEARCH_TERM and type variables to indicate what you want to search for & the type of thing you are searching for. 
 */
public class Main {
    private enum Arguments {
        SEARCH_TERM,
        SEARCH_TYPE;
    }

    public static void main(String[] args) throws IOException {
        String searchTerm = 
            args[Arguments.SEARCH_TERM.ordinal()] != null ? 
                args[Arguments.SEARCH_TERM.ordinal()] 
                : "";
        SearchType searchType = 
            args[Arguments.SEARCH_TYPE.ordinal()] != null ? 
                SearchType.valueOf(args[Arguments.SEARCH_TYPE.ordinal()].toUpperCase()) 
                : SearchType.PATH;

        PathplannerSearch.search(searchTerm, searchType);
    }
}
