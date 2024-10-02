package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import org.trigon.commands.InitExecuteCommand;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class LEDStripCommands {
    public static Command getStaticColorCommand(Color color, LEDStrip... ledStrips) {
        return new StartEndCommand(
                () -> runForLeds((ledStrip -> ledStrip.staticColor(color))),
                () -> {
                },
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getBlinkingCommand(Color color, double blinkingIntervalSeconds, LEDStrip... ledStrips) {
        return new RunCommand(
                () -> runForLeds((ledStrip -> ledStrip.blink(color, blinkingIntervalSeconds))),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getRainbowCommand(LEDStrip... ledStrips) {
        return new RunCommand(
                () -> runForLeds((LEDStrip::rainbow)),
                ledStrips
        ).ignoringDisable(true);
    }

    public static Command getThreeSectionColorCommand(Supplier<Color> firstSectionColor, Supplier<Color> secondSectionColor, Supplier<Color> thirdSectionColor, LEDStrip... ledStrips) {
        return new InitExecuteCommand(
                () -> runForLeds(LEDStrip::clearLedColors),
                () -> runForLeds((ledStrip) -> ledStrip.threeSectionColor(firstSectionColor.get(), secondSectionColor.get(), thirdSectionColor.get())),
                ledStrips
        ).ignoringDisable(true);
    }

    public static void runForLeds(Consumer<LEDStrip> action, LEDStrip... ledStrips) {
        for (LEDStrip ledStrip : ledStrips) {
            action.accept(ledStrip);
        }
    }
}