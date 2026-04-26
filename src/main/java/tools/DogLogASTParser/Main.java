package tools.DogLogASTParser;

import java.nio.file.Files;
import java.nio.file.Path;

import com.github.javaparser.ParserConfiguration;
import com.github.javaparser.StaticJavaParser;
import com.github.javaparser.ast.CompilationUnit;

/**
 * <h2>DogLogASTParser | A tool for replacing SmartDashboard calls with DogLog calls</h2>
 * 
 * <p>The main entry point for DogLogASTParser. It uses an abstract syntax tree (AST) parser called <a href="https://javaparser.org">JavaParser</a> to read and modify the java files in the robot code.
 * 
 * <p>SmartDashboard calls of <code>putNumber</code>, <code>putBoolean</code>, and <code>putString</code> are replaced with <code>DogLog.log</code> calls. <code>SmartDashboard.putData</code> remains unchanged for more complex data.
 * <p>Works with a Github Action workflow (doglog-replacement.yml) to automate the replacement with every push onto the main branch.
 * <p>To execute, manually run the workflow in Github or push to main. It's not recommended to run this tool locally as the purpose of this tool is to automate the replacement and push to the <code>doglog</code> branch for testing. 
 */
public class Main {

    /**
     * <h4>Entry Point</h4>
     * 
     * <p>Walks through the Java files in the robot code, filtering the tools directory, and invokes the {@link #processFile(Path)} method for AST parsing.
     * 
     */
    public static void main(String[] args) {
        StaticJavaParser.getConfiguration()
            .setLanguageLevel(ParserConfiguration.LanguageLevel.JAVA_17);
        final Path root = Path.of("src/main/java");
        try {
            Files.walk(root)
                 .filter(Files::isRegularFile)
                 .filter(p -> !p.toString().contains("tools/"))
                 .filter(p -> p.toString().endsWith(".java"))
                 .forEach(Main::processFile);
        } catch (java.io.IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * <h4>Processes individual Java files.</h4>
     * 
     * <p>Walks through the java files in the robot code, excluding the tools directory, and invokes the {@link #processFile(Path)} method for AST parsing.
     * 
     * <p>It makes a {@link CompilationUnit} for the file, then applies the {@link DogLogRewriter} to replace SmartDashboard calls with DogLog calls.
     * <p>When {@link DogLogRewriter} is done, the modified {@link CompilationUnit} is written back to the file path, replacing the original file.
     * 
     * @param filePath the path of the Java file to be processed
     */
    public static void processFile(Path filePath) {
        try {
            CompilationUnit parsed = StaticJavaParser.parse(filePath);
            DogLogRewriter.applyReplacement(parsed);

            Files.writeString(filePath, parsed.toString());

            System.out.println("Replaced file: " + filePath + " with DogLog calls.");
            
        } catch (Exception e) {
            String errorReason = e.getClass().getSimpleName() + ": " + e.getMessage();
            System.out.println("Error processing file: " + filePath + ", will be skipped: " + errorReason);
        }
    }
}