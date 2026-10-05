package com.stuypulse.robot.util.logged;

import edu.wpi.first.units.Measure;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.LogTable;

/**
 *
 *
 * <h2>Log Table Utility</h2>
 *
 * <p>Utility methods for reading and writing dynamically typed values to and from a {@code
 * LogTable}.
 *
 * <p>This centralizes type handling without having to duplicate the same if else logic in multiple
 * places.
 *
 * @author Faizaan (https://github.com/Faizaan-J)
 */
public final class LogTableUtil {
    private LogTableUtil() {}

    /**
     * Puts a value into a {@code LogTable} with the given key, automatically handling the type of
     * the value.
     *
     * @param table the {@code LogTable} to put the value into
     * @param key The key where the value will be stored.
     * @param value The value to store in the {@code LogTable}. If it's not a supported type, it
     *     will be converted to a string.
     */
    public static void put(LogTable table, String key, Object value) {
        if (value != null && value.getClass().isArray()) {
            putArray(table, key, value);
        } else if (value instanceof Measure<?> measure) {
            table.put(key, measure);
        } else if (value instanceof WPISerializable serializable) {
            table.put(key, serializable);
        } else if (value instanceof Color color) {
            table.put(key, color);
        } else if (value instanceof Boolean) {
            table.put(key, (Boolean) value);
        } else if (value instanceof Double) {
            table.put(key, (Double) value);
        } else if (value instanceof Integer) {
            table.put(key, (Integer) value);
        } else if (value instanceof Long) {
            table.put(key, (Long) value);
        } else if (value instanceof String) {
            table.put(key, (String) value);
        } else if (value instanceof Enum<?> enumerator) {
            putEnum(table, key, enumerator);
        } else if (value != null && value.getClass().isRecord()) {
            table.put(key, (Record) value);
        } else {
            // turn into string for unsupported types, can't be replayed tho.
            table.put(key, value == null ? "null" : value.toString());
        }
    }

    /**
     * Puts an array value into a {@code LogTable}, dispatching to the corresponding array overload.
     * Falls back to a {@code String[]} for element types with no array overload such as a {@code
     * Measure} or boxed wrapper types.
     */
    private static void putArray(LogTable table, String key, Object array) {
        if (array instanceof boolean[] booleanArray) {
            table.put(key, booleanArray);
        } else if (array instanceof int[] intArray) {
            table.put(key, intArray);
        } else if (array instanceof long[] longArray) {
            table.put(key, longArray);
        } else if (array instanceof float[] floatArray) {
            table.put(key, floatArray);
        } else if (array instanceof double[] doubleArray) {
            table.put(key, doubleArray);
        } else if (array instanceof String[] StringArray) {
            table.put(key, StringArray);
        } else if (array instanceof byte[] byteArray) {
            table.put(key, byteArray);
        } else if (array instanceof Enum<?>[] enumArray) {
            putEnumArray(table, key, enumArray);
        } else if (array instanceof StructSerializable[] structSerializableArray) {
            table.put(key, structSerializableArray);
        } else if (array.getClass().getComponentType().isRecord()) {
            table.put(key, (Record[]) array);
        } else {
            Object[] boxed = (Object[]) array;
            String[] strings = new String[boxed.length];
            for (int i = 0; i < boxed.length; i++) {
                strings[i] = String.valueOf(boxed[i]);
            }
            table.put(key, strings);
        }
    }

    /**
     * Gets a value from a {@code LogTable} with the given key, automatically handling the type of
     * the value based on the provided default value.
     *
     * @param <T> The type of the value.
     * @param table The {@code LogTable} to get the value from.
     * @param key The key where the valid is stored.
     * @param defaultValue The default value to return if the key does not exist in the {@code
     *     LogTable}.
     * @return The value from the {@code LogTable} if it exists, otherwise the default value.
     */
    @SuppressWarnings({"unchecked"})
    public static <T> T get(LogTable table, String key, T defaultValue) {
        if (defaultValue != null && defaultValue.getClass().isArray()) {
            return (T) getArray(table, key, defaultValue);
        } else if (defaultValue instanceof Measure<?> measure) {
            return ((T) table.get(key, measure));
        } else if (defaultValue instanceof WPISerializable serializable) {
            return ((T) table.get(key, serializable));
        } else if (defaultValue instanceof Color color) {
            return ((T) table.get(key, color));
        } else if (defaultValue instanceof Boolean bool) {
            return ((T) Boolean.valueOf(table.get(key, bool)));
        } else if (defaultValue instanceof Double dbl) {
            return ((T) Double.valueOf(table.get(key, dbl)));
        } else if (defaultValue instanceof Integer integer) {
            return ((T) Integer.valueOf(table.get(key, integer)));
        } else if (defaultValue instanceof Long lng) {
            return ((T) Long.valueOf(table.get(key, lng)));
        } else if (defaultValue instanceof String string) {
            return ((T) table.get(key, string));
        } else if (defaultValue instanceof Enum<?> enumerator) {
            return ((T) getEnum(table, key, enumerator));
        } else if (defaultValue != null && defaultValue.getClass().isRecord()) {
            return ((T) table.get(key, ((Record) defaultValue)));
        }

        return defaultValue;
    }

    /**
     * Gets an array value from a {@code LogTable}, dispatching to the corresponding array overload.
     * Types with no array overload such as {@code Measure} or boxed wrapper types aren't
     * replayable, so the default array is returned back.
     */
    private static Object getArray(LogTable table, String key, Object defaultArray) {
        if (defaultArray instanceof boolean[] booleanArray) {
            return table.get(key, booleanArray);
        } else if (defaultArray instanceof int[] intArray) {
            return table.get(key, intArray);
        } else if (defaultArray instanceof long[] longArray) {
            return table.get(key, longArray);
        } else if (defaultArray instanceof float[] floatArray) {
            return table.get(key, floatArray);
        } else if (defaultArray instanceof double[] doubleArray) {
            return table.get(key, doubleArray);
        } else if (defaultArray instanceof String[] stringArray) {
            return table.get(key, stringArray);
        } else if (defaultArray instanceof byte[] byteArray) {
            return table.get(key, byteArray);
        } else if (defaultArray instanceof Enum<?>[] enumArray) {
            return getEnumArray(table, key, enumArray);
        } else if (defaultArray instanceof StructSerializable[] structSerializableArray) {
            return table.get(key, structSerializableArray);
        } else if (defaultArray.getClass().getComponentType().isRecord()) {
            return table.get(key, (Record[]) defaultArray);
        }

        return defaultArray;
    }

    @SuppressWarnings("unchecked")
    private static <E extends Enum<E>> void putEnum(LogTable table, String key, Enum<?> value) {
        table.put(key, (E) value);
    }

    @SuppressWarnings("unchecked")
    private static <E extends Enum<E>> E getEnum(LogTable table, String key, Enum<?> defaultValue) {
        return table.get(key, (E) defaultValue);
    }

    @SuppressWarnings("unchecked")
    private static <E extends Enum<E>> void putEnumArray(
            LogTable table, String key, Enum<?>[] value) {
        table.put(key, (E[]) value);
    }

    @SuppressWarnings("unchecked")
    private static <E extends Enum<E>> E[] getEnumArray(
            LogTable table, String key, Enum<?>[] defaultValue) {
        return table.get(key, (E[]) defaultValue);
    }

    /**
     * Returns true if the value is a supported type for logging, false otherwise.
     *
     * @param value The value to check for logging support.
     * @return true if the value is a supported type for logging, false otherwise.
     */
    public static boolean isSupportedType(Object value) {
        if (value == null) {
            return false;
        }
        if (value.getClass().isArray()) {
            return value instanceof boolean[]
                    || value instanceof int[]
                    || value instanceof long[]
                    || value instanceof float[]
                    || value instanceof double[]
                    || value instanceof String[]
                    || value instanceof byte[]
                    || value instanceof Enum<?>[]
                    || value instanceof StructSerializable[]
                    || value.getClass().getComponentType().isRecord();
        }
        return value instanceof Measure<?>
                || value instanceof WPISerializable
                || value instanceof Color
                || value instanceof Boolean
                || value instanceof Double
                || value instanceof Integer
                || value instanceof Long
                || value instanceof String
                || value instanceof Enum<?>
                || value.getClass().isRecord();
    }
}
