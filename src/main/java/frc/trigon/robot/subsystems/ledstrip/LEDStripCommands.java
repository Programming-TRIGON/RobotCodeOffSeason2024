package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.InitExecuteCommand;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class LEDStripCommands {
    public static Command getStaticColorCommand(Color color, LEDStrip... ledStrips) {
        return new StartEndCommand(
                () -> {
                },
                () -> runForLEDs((ledStrip -> ledStrip.staticColor(color)), ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getBlinkingCommand(Color color, double blinkingIntervalSeconds, LEDStrip... ledStrips) {
        return new ExecuteEndCommand(
                () -> runForLEDs((ledStrip -> ledStrip.blink(color, blinkingIntervalSeconds)), ledStrips),
                () -> runForLEDs((LEDStrip::clearLedColors)),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getRainbowCommand(LEDStrip... ledStrips) {
        return new RunCommand(
                () -> runForLEDs((LEDStrip::rainbow), ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getBreathingCommand(Color color, int breathingLEDs, double cycleTimeSeconds, LEDStrip... ledStrips) {
        return new RunCommand(
                () -> runForLEDs((ledStrip) -> ledStrip.breathe(color, breathingLEDs, cycleTimeSeconds), ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getSingleBreatheCommand(Color color, int breathingLEDs, double timeSeconds, LEDStrip... ledStrips) {
        return new RunCommand(
                () -> runForLEDs((ledStrip) -> ledStrip.singleBreathe(color, breathingLEDs, timeSeconds)),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getThreeSectionColorCommand(Supplier<Color> firstSectionColor, Supplier<Color> secondSectionColor, Supplier<Color> thirdSectionColor, LEDStrip... ledStrips) {
        return new InitExecuteCommand(
                () -> runForLEDs(LEDStrip::clearLedColors, ledStrips),
                () -> runForLEDs((ledStrip) -> ledStrip.threeSectionColor(firstSectionColor.get(), secondSectionColor.get(), thirdSectionColor.get()), ledStrips),
                ledStrips
        ).ignoringDisable(true);
    }

    public static void runForLEDs(Consumer<LEDStrip> action, LEDStrip... ledStrips) {
        for (LEDStrip ledStrip : ledStrips)
            action.accept(ledStrip);
    }
}