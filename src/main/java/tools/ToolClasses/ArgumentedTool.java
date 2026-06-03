/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package tools.ToolClasses;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * <h2>ArgumentedTool</h2>
 * A class that represents a tool that takes in arguments.
 * <p>Using gradle tasks with arguments, it automatically parses the arguments based on an enum that the user defines and passes in.
 * <p>The enum must implement the {@link ArgumentEnum} interface, which must specify a type via {@link ArgumentEnum#getArgumentType()}, 
 * default value (can be null) via {@link ArgumentEnum#getDefaultValue()}, and description for each argument via {@link ArgumentEnum#getDescription()}.
 * The order of the arguments is determined by the order of the enum constants.
 * 
 * <p>When parsing, it does the following:
 * <ol>
 *  <li>Splits the raw arguments by spaces, unless they are within quotes. (e.g. <code>-Pargs="arg1 'arg 2' arg3"</code> => <code>[arg1, 'arg 2', arg3]</code>)</li>
 *  <li>Removes the surrounding quotes from the split arguments (e.g. <code>[arg1, 'arg 2', arg3]</code> => <code>[arg1, arg 2, arg3]</code>)</li>
 *  <li>Iterates through the split arguments and attempts to parse them into the type specified by the enum constant {@link ArgumentEnum#getArgumentType()}.
 *    <ul>
 *      <li>If the type is a string, then it will stay as is.</li>
 *      <li>If the type is an integer, then it will attempt to parse it into an integer.</li>
 *      <li>If the type is a double, then it will attempt to parse it into a double.</li>
 *      <li>If the type is a boolean, then it will attempt to parse it into a boolean.</li>
 *      <li>If the type is an enum, then it will attempt to parse it into the enum by matching the name of the enum constant (case-insensitive).</li>
 *    </ul>
 *  </li>
 *   <li>If the argument is missing or fails to parse, then it will use the default value specified by the enum. 
 *   If there is no default value and the argument fails to parse, then it will throw an exception.</li>
 *  <li>Extra arguments that are specified, greater than the length of the enum constant, will be ignored.</li>
 * </ol>
 * <p>The constructor takes in an enum class that defines the arguments. To use the, the {@link #execute(Map)} method must be implemented.
 * This method has the parsed arguments passed in as a map, where the keys are the from the argument enum constants, and the values are the parsed arguments
 * from the raw string passed into the {@link #run(String)} method.
 * 
 * <p>The {@link #run(String)} method is the main entry point for the tool. This will automatically parse the raw string arguments and call the 
 * {@link #execute(Map)} method with the parsed arguments. 
 */
public abstract class ArgumentedTool<E extends Enum<E> & ArgumentEnum> {
    private final Class<E> argumentsEnumClass;

    @SuppressWarnings("unchecked")
    private Map<E, Object> parseArguments(String rawArgs) {
        String[] splitArgs = rawArgs.split(" (?=(?:[^\"']*[\"'][^\"']*[\"'])*[^\"']*$)"); // split by spaces if not within single or double quotes
                                                                                               // all regex statements in this repo are vibe coded
        Object[] argumentValues = new Object[splitArgs.length];
        for (int i = 0; i < splitArgs.length; i++) {
            argumentValues[i] = splitArgs[i].replaceAll("^\"|\"$|^'|'$", ""); // remove surrounding quotes
            
            E currentEnum = argumentsEnumClass.getEnumConstants()[i];
            if (currentEnum.getArgumentType() == String.class) continue;

            switch (argumentsEnumClass.getEnumConstants()[i].getArgumentType().getSimpleName()) {
                case "Integer", "int" -> {
                    try {
                        argumentValues[i] = Integer.parseInt((String) argumentValues[i]);
                    } catch (Exception e) {
                        if (currentEnum.getDefaultValue() == null || currentEnum.getDefaultValue().getClass() != Integer.class) {
                            throw new IllegalArgumentException("Argument " + currentEnum.name() + "(" + currentEnum.getDescription() + ") must be an integer");
                        }
                    }
                    break;
                }
                case "Double", "double" -> {
                    try {
                        argumentValues[i] = Double.parseDouble((String) argumentValues[i]);
                    } catch (Exception e) {
                        if (currentEnum.getDefaultValue() == null || currentEnum.getDefaultValue().getClass() != Double.class) {
                            throw new IllegalArgumentException("Argument #" + (i + 1) + " (" + currentEnum.getDescription() + ") must be a double");
                        }
                    }
                    break;
                }
                case "Boolean", "boolean" -> {
                    if (((String) argumentValues[i]).equalsIgnoreCase("true") || ((String) argumentValues[i]).equalsIgnoreCase("false")) {
                        argumentValues[i] = Boolean.parseBoolean((String) argumentValues[i]);
                    } else {
                        if (currentEnum.getDefaultValue() == null || currentEnum.getDefaultValue().getClass() != Boolean.class) {
                            throw new IllegalArgumentException("Argument #" + (i + 1) + " (" + currentEnum.getDescription() + ") must be a boolean");
                        }
                    }
                    break;
                }
                default -> {
                    if (currentEnum.getArgumentType().isEnum()) {
                        Enum<E>[] enumConstants = (Enum<E>[]) currentEnum.getArgumentType().getEnumConstants();
                        try {
                            String enumName = (String) argumentValues[i];
                            Enum<?> match = null;
                            for (Enum<?> c : (Enum<?>[]) currentEnum.getArgumentType().getEnumConstants()) {
                                if (c.name().equalsIgnoreCase(enumName)) {
                                    match = c;
                                    break;
                                }
                            }

                            if (match == null) {
                                throw new IllegalArgumentException(); // to be caught by catch block
                            }
                            argumentValues[i] = match;
                        } catch (Exception e) {
                            throw new IllegalArgumentException("Argument #" + (i + 1) + " (" + currentEnum.getDescription() + ") must be one of " + Arrays.toString(enumConstants));
                        }
                    } else {
                        argumentValues[i] = currentEnum.getDefaultValue();
                    }
                }
            }
        }

        Map<E, Object> parsedArgs = new HashMap<>();
        for (int i = 0; i < argumentsEnumClass.getEnumConstants().length; i++) {
            E currentEnum = argumentsEnumClass.getEnumConstants()[i];
            if (i < argumentValues.length) {
                parsedArgs.put(currentEnum, argumentValues[i]);
            } else {
                parsedArgs.put(currentEnum, currentEnum.getDefaultValue());
            }
        }

        System.out.println(parsedArgs + " - Parsed Args");
        return parsedArgs;
    }

    protected ArgumentedTool(Class<E> argumentsEnumClass) {
        this.argumentsEnumClass = argumentsEnumClass;
    }

    /**
     * The functionality for the tool. Must be implemented.
     * @param parsedArgs the parsed arguments
     */
    protected abstract void execute(Map<E, Object> parsedArgs);
    /**
     * The main entry point for the tool. This will call the {@link #execute(Map)} method with the parsed arguments.
     */
    public final void run(String rawArgs) {
        Map<E, Object> parsedArgs = parseArguments(rawArgs);
        execute(parsedArgs);
    }
}
