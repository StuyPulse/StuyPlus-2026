package tools.PathplannerFlip.PathFlip;

/**
 *
 *
 * <h2>Flip paths of your choice across the y coordinate axis</h2>
 *
 * <p>
 * To use, run:
 *
 * <pre>
 * ./gradlew runPathplannerPathFlip -Pargs="[target paths]"
 * </pre>
 * 
 * Split each path you want to flip with spaces and if a path has multiple words, put it in quotes
 * 
 * <p>
 * <b>Example:</b>
 *
 * <pre>
 * ./gradlew runPathplannerPathFlip -Pargs="'RB to Mid Neutral LF' 'Mid Neutral LF to RB Shoot'"
 * </pre>
 *
 * <p>
 * <b>Arguments:</b>
 *
 * <ul>
 * <li><b>[target paths]</b>: The paths to flip over the y axis. This is variadic and can take in as many paths as needed.
 * </ul>
 */
public class Main {
    public static void main(String[] args) {
        new PathplannerPathFlip().run(args[0]);
    }   
}
