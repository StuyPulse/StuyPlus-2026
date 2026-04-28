package tools.PathplannerSearch;

import java.io.IOException;
import tools.PathplannerSearch.PathplannerSearch.SearchType;

/*
 * todo: document usage
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