package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.InitExecuteCommand;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class LEDStripCommands {
    public static Command getStaticColorCommand(Color color, LEDStrip... ledStrips) {
        return new ExecuteEndCommand(
                () -> runForLEDs((LEDStrip -> LEDStrip.staticColor(color)), ledStrips),
                () -> runForLEDs(LEDStrip::clearLedColors, ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getBlinkingCommand(Color firstColor, Color secondColor, double blinkingIntervalSeconds, LEDStrip... ledStrips) {
        return new ExecuteEndCommand(
                () -> runForLEDs((LEDStrip -> LEDStrip.blink(firstColor, secondColor, blinkingIntervalSeconds)), ledStrips),
                () -> runForLEDs(LEDStrip::clearLedColors, ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getRainbowCommand(LEDStrip... ledStrips) {
        return new ExecuteEndCommand(
                () -> runForLEDs((LEDStrip::rainbow), ledStrips),
                () -> runForLEDs(LEDStrip::clearLedColors, ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getBreatheCommand(Color color, int breathingLEDs, double cycleTimeSeconds, boolean shouldLoop, LEDStrip... ledStrips) {
        return new ExecuteEndCommand(
                () -> runForLEDs((LEDStrip) -> LEDStrip.breathe(color, breathingLEDs, cycleTimeSeconds, shouldLoop), ledStrips),
                () -> {
                    runForLEDs(LEDStrip::resetLEDSettings, ledStrips);
                    runForLEDs(LEDStrip::clearLedColors, ledStrips);
                },
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getThreeSectionColorCommand(Supplier<Color> firstSectionColor, Supplier<Color> secondSectionColor, Supplier<Color> thirdSectionColor, LEDStrip... ledStrips) {
        return new InitExecuteCommand(
                () -> runForLEDs(LEDStrip::clearLedColors, ledStrips),
                () -> runForLEDs((LEDStrip) -> LEDStrip.threeSectionColor(firstSectionColor.get(), secondSectionColor.get(), thirdSectionColor.get()), ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static void runForLEDs(Consumer<LEDStrip> action, LEDStrip... ledStrips) {
        for (LEDStrip LEDStrip : ledStrips)
            action.accept(LEDStrip);
    }
}