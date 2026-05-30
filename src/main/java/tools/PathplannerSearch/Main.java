/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package tools.PathplannerSearch;

import java.io.IOException;

import tools.PathplannerSearch.PathplannerSearch.SearchType;
import tools.ToolClasses.ArgumentEnum;

/**
 *
 *
 * <h2>Main class for PathplannerSearch</h2>
 *
 * <p>
 * To use, run:
 *
 * <pre>
 * ./gradlew runPathplannerSearch -Pargs="[search term] [search type]"
 * </pre>
 *
 * <p>
 * <b>Example:</b>
 *
 * <pre>
 * ./gradlew runPathplannerSearch -Pargs="disruptAUton path"
 * </pre>
 *
 * If your search term has spaces, then wrap it in quotes:
 *
 * <p>
 * <b>Example:</b>
 *
 * <pre>
 * ./gradlew runPathplannerSearch -Pargs="'Disrupt Auton Thing' path"
 * </pre>
 *
 * <p>
 * <b>Arguments:</b>
 *
 * <ul>
 * <li><b>[search term]</b>: The term to search for in the Pathplanner files.
 * <li><b>[search type]</b>: The type of search to perform:
 * <ul>
 * <li>{@code linked_waypoint}
 * <li>{@code path}
 * </ul>
 * </ul>
 */
public class Main {
    public enum ARGUMENTS implements ArgumentEnum {
        SEARCH_TERM(String.class, "", "The term to search for in the Pathplanner files"),
        SEARCH_TYPE(SearchType.class, SearchType.PATH, "The type of search to perform.");

        private final Class<?> argumentType;
        private final Object defaultValue;
        private final String description;

        ARGUMENTS(Class<?> argumentType, Object defaultValue, String description) {
            this.argumentType = argumentType;
            this.defaultValue = defaultValue;
            this.description = description;
        }

        @Override
        public Class<?> getArgumentType() {
            return argumentType;
        }

        @Override
        public Object getDefaultValue() {
            return defaultValue;
        }

        @Override
        public String getDescription() {
            return description;
        }
    }

    public static void main(String[] args) throws IOException {
        new PathplannerSearch().run(args[0]);
    }
}
