/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package tools.ToolClasses;

import java.util.ArrayList;
import java.util.List;

public abstract class VarArgumentedTool<V> {
    private final Class<?> argumentType;
    private final String description;

    @SuppressWarnings("unchecked")
    private List<V> parseArguments(String rawArgs) {
        String[] argStrings = rawArgs.split(" (?=(?:[^\"']*[\"'][^\"']*[\"'])*[^\"']*$)"); // split by spaces if not within single or double quotes
        List<V> arguments = new ArrayList<V>();
        for (int i = 0; i < argStrings.length; i++) {
            argStrings[i] = argStrings[i].replaceAll("^\"|\"$|^'|'$", ""); // remove surrounding quotes
            if (this.argumentType == String.class) {
                arguments.add(((V) argStrings[i]));
                continue;
            }

            switch (this.argumentType.getSimpleName()) {
                case "Integer", "int" -> {
                    try {
                        arguments.add((V) Integer.valueOf(argStrings[i]));
                    } catch (Exception e) {
                        System.err.println("Failed to parse argument " + argStrings[i] + " into an integer. Skipping.");
                    }
                }
                case "Double", "double" -> {
                    try {
                        arguments.add((V) Double.valueOf(argStrings[i]));
                    } catch (Exception e) {
                        System.err.println("Failed to parse argument " + argStrings[i] + " into a double. Skipping.");
                    }
                }
                case "Boolean", "boolean" -> {
                    if (argStrings[i].equalsIgnoreCase("true") || argStrings[i].equalsIgnoreCase("false")) {
                        arguments.add((V) Boolean.valueOf(argStrings[i]));
                    } else {
                        System.err.println("Failed to parse argument " + argStrings[i] + " into a boolean. Skipping.");
                    }
                }
                default -> {
                    if (this.argumentType.isEnum()) {
                        try {
                            String enumName = (String) argStrings[i];
                            Enum<?> match = null;
                            for (Enum<?> c : (Enum<?>[]) this.argumentType.getEnumConstants()) {
                                if (c.name().equalsIgnoreCase(enumName)) {
                                    match = c;
                                    break;
                                }
                            }

                            if (match == null) {
                                throw new IllegalArgumentException(); // to be caught by catch block
                            }
                            arguments.add((V) match);
                        } catch (Exception e) {
                            System.err.println("Failed to parse argument " + argStrings[i] + " into an enum of type " + this.argumentType.getSimpleName() + ". Skipping.");
                        }
                    } else {
                        System.err.println("Unsupported argument type: " + this.argumentType.getSimpleName());
                    }
                }
            }
        }

        return arguments;
    }

    protected VarArgumentedTool(Class<?> argumentType, String description) {
        this.argumentType = argumentType;
        this.description = description;
    }

    protected abstract void execute(List<V> arguments);

    public final void run(String rawArgs) {
        List<V> arguments = parseArguments(rawArgs);
        execute(arguments);
    }
}
