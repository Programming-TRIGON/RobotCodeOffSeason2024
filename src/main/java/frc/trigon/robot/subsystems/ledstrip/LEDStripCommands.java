package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.InitExecuteCommand;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class LEDStripCommands {
    public static Command getStaticColorCommand(Color color, LEDStrip... ledStrips) {
        return new ExecuteEndCommand(
                () -> runForLEDs((LEDStrip -> LEDStrip.staticColor(color)), ledStrips),
                () -> runForLEDs(LEDStrip::clearLEDColors, ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getBlinkingCommand(Color firstColor, Color secondColor, double blinkingIntervalSeconds, LEDStrip... ledStrips) {
        return new ExecuteEndCommand(
                () -> runForLEDs((LEDStrip -> LEDStrip.blink(firstColor, secondColor, blinkingIntervalSeconds)), ledStrips),
                () -> runForLEDs(LEDStrip::clearLEDColors, ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getRainbowCommand(LEDStrip... ledStrips) {
        return new ExecuteEndCommand(
                () -> runForLEDs((LEDStrip::rainbow), ledStrips),
                () -> runForLEDs(LEDStrip::clearLEDColors, ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getBreatheCommand(Color color, int breathingLEDs, double cycleTimeSeconds, boolean shouldLoop, boolean inverted, LEDStrip... ledStrips) {
        return new FunctionalCommand(
                () -> {
                    if (!shouldLoop)
                        runForLEDs(LEDStrip::resetLEDSettings, ledStrips);
                },
                () -> runForLEDs((LEDStrip) -> LEDStrip.breathe(color, breathingLEDs, cycleTimeSeconds, shouldLoop, inverted), ledStrips),
                (interrupted) -> runForLEDs(LEDStrip::clearLEDColors, ledStrips),
                () -> false,
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getColorFlowCommand(Color color, double cycleTimeSeconds, boolean shouldLoop, boolean inverted, LEDStrip... ledStrips) {
        return new FunctionalCommand(
                () -> {
                    if (!shouldLoop)
                        runForLEDs(LEDStrip::resetLEDSettings, ledStrips);
                },
                () -> runForLEDs((LEDStrip) -> LEDStrip.colorFlow(color, cycleTimeSeconds, shouldLoop, inverted), ledStrips),
                (interrupted) -> runForLEDs(LEDStrip::clearLEDColors, ledStrips),
                () -> false,
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getAlternateColorCommand(Color firstColor, Color secondColor, double intervalSeconds, LEDStrip... ledStrips) {
        return new ExecuteEndCommand(
                () -> runForLEDs((LEDStrip -> LEDStrip.alternateColor(firstColor, secondColor, intervalSeconds)), ledStrips),
                () -> runForLEDs(LEDStrip::clearLEDColors, ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getSectionColorCommand(int amountOfSections, Supplier<Color>[] colors, LEDStrip... ledStrips) {
        return new InitExecuteCommand(
                () -> runForLEDs(LEDStrip::clearLEDColors, ledStrips),
                () -> runForLEDs((LEDStrip) -> LEDStrip.sectionColor(amountOfSections, colors), ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static void runForLEDs(Consumer<LEDStrip> action, LEDStrip... ledStrips) {
        for (LEDStrip LEDStrip : ledStrips)
            action.accept(LEDStrip);
    }
}