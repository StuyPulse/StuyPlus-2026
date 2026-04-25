package tools.DogLogASTParser;

import com.github.javaparser.ast.CompilationUnit;
import com.github.javaparser.ast.expr.MethodCallExpr;
import com.github.javaparser.ast.expr.NameExpr;
import com.github.javaparser.ast.visitor.ModifierVisitor;

public class DogLogRewriter extends ModifierVisitor<Void> {

    private boolean smartDashboardCallFound = false;

    @Override
    public MethodCallExpr visit(MethodCallExpr methodCall, Void arg) {
        super.visit(methodCall, arg);

        if (!methodCall.getScope().isPresent()) {
            return methodCall;
        }

        if (methodCall.getScope().get().toString().equals("SmartDashboard")) {
            smartDashboardCallFound = true;
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
        }

        return methodCall;
    }

    public static void applyReplacement(CompilationUnit parsed) {
        DogLogRewriter doglogRewriter = new DogLogRewriter();
        parsed.accept(doglogRewriter, null);

        if (!doglogRewriter.smartDashboardCallFound) {
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