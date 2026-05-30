/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package tools.ToolClasses;

/**
 * <h2>ArgumentEnum</h2>
 * An interface that represents an enum that defines arguments for an {@link ArgumentedTool}.
 * <p>Each enum constant represents an argument. It must specify the following:
 * <ul>
 *    <li><b>ArgumentType</b>: The type of the argument. This takes in a class and is used to parse the raw string argument into the correct type. Supported types are:
 *    <ul>
 *      <li>{@link String}</li>
 *      <li>{@link Integer}</li>
 *      <li>{@link Double}</li>
 *      <li>{@link Boolean}</li>
 *      <li>{@link Enum}</li>
 *    </ul>
 *    </li>
 *  <li><b>DefaultValue</b>: The default value for the argument. This is used if the argument is missing or fails to parse. Can be null.</li>
 *  <li><b>Description</b>: A description of the argument. This is used for documentation purposes.</li>
 * </ul>
 * <p>Example usage:
 * <pre>
 * public enum ARGUMENTS implements ArgumentEnum {
 *     TIMEOUT(Integer.class, 30, "Timeout in seconds"),
 *     VERBOSE(Boolean.class, false, "Enable verbose logging"),
 *
 *     private final Class{@literal <?>} argumentType;
 *     private final Object defaultValue;
 *     private final String description;
 *
 *     ARGUMENTS({@literal Class<?>} argumentType, Object defaultValue, String description) {
 *         this.argumentType = argumentType;
 *         this.defaultValue = defaultValue;
 *         this.description = description;
 *     }
 *
 *     {@literal @Override}
 *     public Class{@literal <?>} getArgumentType() { return argumentType; }
 *
 *     {@literal @Override}
 *     public Object getDefaultValue() { return defaultValue; }
 *
 *     {@literal @Override}
 *     public String getDescription() { return description; }
 * }
 * </pre>
 */
public interface ArgumentEnum {
    /**
     * Gets the argument type.
     * @return the argument type
     */
    Class<?> getArgumentType();
    /**
     * Gets the default value for the argument. This is used if the argument is missing or fails to parse. Can be null.
     * @return the default value
     */
    Object getDefaultValue();
    /**
     * Gets the description of the argument. This is used for documentation purposes.
     * @return the description
     */
    String getDescription();
}
