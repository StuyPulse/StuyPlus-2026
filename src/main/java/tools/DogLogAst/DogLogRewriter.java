package tools.DogLogAST;

import com.github.javaparser.ast.CompilationUnit;
import com.github.javaparser.ast.expr.MethodCallExpr;
import com.github.javaparser.ast.expr.NameExpr;
import com.github.javaparser.ast.visitor.ModifierVisitor;

public class DogLogRewriter extends ModifierVisitor<Void> {

    @Override
    public MethodCallExpr visit(MethodCallExpr methodCall, Void arg) {
        super.visit(methodCall, arg);

        if (!methodCall.getScope().isPresent()) {
            return methodCall;
        }

        if (methodCall.getScope().get().toString().equals("SmartDashboard")) {
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
        parsed.accept(new DogLogRewriter(), null);

        String doglogImport = "dev.doglog.DogLog";
        boolean hasDogLogImport = parsed.getImports().stream()
            .anyMatch(i -> i.getNameAsString().equals(doglogImport));

        if (!hasDogLogImport) {
            parsed.addImport(doglogImport);
        }
    }
}