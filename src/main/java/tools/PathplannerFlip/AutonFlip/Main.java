package tools.PathplannerFlip.AutonFlip;

/**
 *
 *
 * <h2>Flip autons of your choice across the y coordinate axis</h2>
 *
 * <p>
 * To use, run:
 *
 * <pre>
 * ./gradlew runPathplannerAutonFlip -Pargs="[target autons]"
 * </pre>
 * 
 * Split each auton you want to flip with spaces and if an auton has multiple words, put it in quotes
 * 
 * <p>
 * <b>Example:</b>
 *
 * <pre>
 * ./gradlew runPathplannerAutonFlip -Pargs="'RB 1 Sweep Angled' 'Front Hub Shoot Preloads'"
 * </pre>
 *
 * <p>
 * <b>Arguments:</b>
 *
 * <ul>
 * <li><b>[target autons]</b>: The autons to flip over the y axis. This is variadic and can take in as many autons as needed.
 * </ul>
 */
public class Main {
    public static void main(String[] args) {
        new PathplannerAutonFlip().run(args[0]);
    }
}
