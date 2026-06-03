/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package tools.ToolClasses;

/**
 * <h2>Tool</h2>
 * <p>A class that represents a tool that can be run. This is for simple actions and doesn't take in any arguments.
 * For tools that take in arguments, use {@link ArgumentedTool} instead.
 * <p>To use, simply extend this class and implement the {@link #execute()} method. Then, to run the tool, simply call the {@link #run()} method.
 */
public abstract class Tool {
    /**
     * Default constructor for the tool. This is protected to prevent instantiation of the tool without extending it.
     */
    protected Tool() {

    }

    /**
     * The functionality for the tool. Must be implemented.
     */
    protected abstract void execute();
    /**
     * The main entry point for the tool. This will call the {@link #execute()} method.
     */
    public final void run() {
        execute();
    }
}
