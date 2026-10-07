import edu.mit.csail.sdg.alloy4.A4Reporter;
import edu.mit.csail.sdg.alloy4.Err;
import edu.mit.csail.sdg.ast.Command;
import edu.mit.csail.sdg.ast.Module;
import edu.mit.csail.sdg.parser.CompUtil;
import edu.mit.csail.sdg.translator.A4Options;
import edu.mit.csail.sdg.translator.A4Solution;
import edu.mit.csail.sdg.translator.TranslateAlloyToKodkod;

public final class SovereigntyConcentrationAlloyRunner {
    private static String json(String value) {
        return value.replace("\\", "\\\\").replace("\"", "\\\"");
    }

    public static void main(String[] args) {
        if (args.length != 1) {
            System.err.println("usage: SovereigntyConcentrationAlloyRunner <model.als>");
            System.exit(2);
        }
        try {
            Module world = CompUtil.parseEverything_fromFile(A4Reporter.NOP, null, args[0]);
            A4Options options = new A4Options();
            options.solver = A4Options.SatSolver.SAT4J;
            int index = 0;
            for (Command command : world.getAllCommands()) {
                index++;
                A4Solution solution = TranslateAlloyToKodkod.execute_command(
                    A4Reporter.NOP, world.getAllReachableSigs(), command, options);
                String actual = solution.satisfiable() ? "SAT" : "UNSAT";
                System.out.println(
                    "{\"index\":" + index +
                    ",\"label\":\"" + json(command.label) +
                    "\",\"check\":" + command.check +
                    ",\"expects\":" + command.expects +
                    ",\"actual\":\"" + actual +
                    "\",\"command\":\"" + json(command.toString()) + "\"}"
                );
            }
        } catch (Throwable error) {
            error.printStackTrace(System.err);
            System.exit(3);
        }
    }
}
