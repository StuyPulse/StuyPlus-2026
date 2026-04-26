package tools.DogLogASTParser;

import com.github.javaparser.ast.CompilationUnit;
import com.github.javaparser.ast.expr.MethodCallExpr;
import com.github.javaparser.ast.expr.NameExpr;
import com.github.javaparser.ast.visitor.ModifierVisitor;

/**
 * <h2>DogLogRewriter | Actual functionality</h2>
 * 
 * <p>The actual class that performs the AST parsing and replacement of <code>SmartDashboard</code> calls with <code>DogLog</code> calls, invoked by the {@link Main} class.
 * 
 * <p>Extends {@link ModifierVisitor} to "visit" and modify the calls.
 */
public class DogLogRewriter extends ModifierVisitor<Void> {

    private boolean fileChanged = false;

    /**
     * <h4>Visit Method</h4>
     * 
     * <p>Overrides the {@link visit} method for method calls. 
     * <p>It loops through every {@link MethodCallExpr} to check for a <code>SmartDashboard</code> call with the methods <code>putNumber</code>, <code>putBoolean</code>, or <code>putString</code>. If so, it replaces the scope with <code>DogLog</code> and changes the method name to <code>log</code>.
     * 
     * <p>Sets variable <code>fileChanged</code> to check if any changes were made to the file to determine whether to import <code>DogLog</code> at the end of the file.
     * 
     * @param methodCall the method call expression being visited
     * @param arg additional argument (not used in this case)
     * @return the modified method call expression
     */
    @Override
    public MethodCallExpr visit(MethodCallExpr methodCall, Void arg) {
        super.visit(methodCall, arg);

        if (!methodCall.getScope().isPresent()) {
            return methodCall;
        }

        boolean isSmartDashboardCall = methodCall.getScope()
            .filter(s -> s instanceof NameExpr)
            .map(s -> ((NameExpr) s).getNameAsString())
            .filter(name -> name.equals("SmartDashboard"))
            .isPresent();

        if (isSmartDashboardCall ) {
            String methodName = methodCall.getNameAsString();
            boolean shouldBeReplaced = 
                switch (methodName) {
                    case "putNumber", "putBoolean", "putString" -> true;
                    default -> false;
            };

            if (!shouldBeReplaced) {
                return methodCall;
            }

            methodCall.setScope(new NameExpr("DogLog"));
            methodCall.setName("log");
            fileChanged = true;
        }

        return methodCall;
    }

    /**
     * <h4>Applies the replacement</h4>
     * 
     * <p>Applies the <code>SmartDashboard</code> to <code>DogLog</code> replacement to the given {@link CompilationUnit}.
     * 
     * <p>If the <code>fileChanged</code> flag is true, it checks if the files already has an import for <code>DogLog</code>. If not, it adds the import statement. If the flag is false, it does nothing to avoid unnecessary changes and imports.
     * 
     * @param parsed the {@link CompilationUnit} to process
     */
    public static void applyReplacement(CompilationUnit parsed) {
        DogLogRewriter doglogRewriter = new DogLogRewriter();
        parsed.accept(doglogRewriter, null);

        if (!doglogRewriter.fileChanged) {
            return;
        }

        String doglogImport = "dev.doglog.DogLog";
        boolean hasDogLogImport = parsed.getImports().stream()
            .anyMatch(i -> i.getNameAsString().equals(doglogImport));

        if (!hasDogLogImport) {
            parsed.addImport(doglogImport);
        }
    }
}