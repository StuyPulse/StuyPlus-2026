package tools.doglog_ast;

import java.nio.file.Files;
import java.nio.file.Path;

import com.github.javaparser.StaticJavaParser;
import com.github.javaparser.ast.CompilationUnit;

public class Main {
    public static void main(String[] args) {
        Path root = Path.of("src/main/java");
        try {
            Files.walk(root)
                 .filter(Files::isRegularFile)
                 .filter(p -> !p.toString().contains("tools/doglog_ast"))
                 .filter(p -> p.toString().endsWith(".java"))
                 .forEach(Main::processFile);
        } catch (java.io.IOException e) {
            e.printStackTrace();
        }
    }

    public static void processFile(Path filePath) {
        try {
            CompilationUnit parsed = StaticJavaParser.parse(filePath);
            DogLogRewriter.applyReplacement(parsed);

            Files.writeString(filePath, parsed.toString());

            System.out.println("Replaced file: " + filePath + " with DogLog calls.");
            
        } catch (java.io.IOException e) {
            e.printStackTrace();
        }
    }
}
