package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import org.trigon.commands.InitExecuteCommand;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class LEDStripCommands {
    public static Command getStaticColorCommand(Color color, LEDStrip ledStrip) {
        return new StartEndCommand(
                () -> ledStrip.staticColor(color),
                () -> {
                },
                ledStrip
        ).ignoringDisable(true);
    }

    public static Command getStaticColorCommand(Color color) {
        return new StartEndCommand(
                () -> runForAllLeds((ledStrip) -> ledStrip.staticColor(color)),
                () -> {
                },
                LEDStripConstants.RIGHT_CLIMBER_LEDS, LEDStripConstants.LEFT_CLIMBER_LEDS
        ).ignoringDisable(true);
    }

    public static Command getBlinkingCommand(Color color, boolean isFastBlinkingEnabled, LEDStrip ledStrip) {
        return new RunCommand(
                () -> ledStrip.blink(color, isFastBlinkingEnabled),
                ledStrip
        ).ignoringDisable(true);
    }

    public static Command getBlinkingCommand(Color color, boolean isFastBlinkingEnabled) {
        return new RunCommand(
                () -> runForAllLeds((ledStrip) -> ledStrip.blink(color, isFastBlinkingEnabled)),
                LEDStripConstants.RIGHT_CLIMBER_LEDS, LEDStripConstants.LEFT_CLIMBER_LEDS
        ).ignoringDisable(true);
    }

    public static Command getRainbowCommand(LEDStrip ledStrip) {
        return new RunCommand(
                ledStrip::rainbow,
                ledStrip
        ).ignoringDisable(true);
    }

    public static Command getRainbowCommand() {
        return new RunCommand(
                () -> runForAllLeds((LEDStrip::rainbow)),
                LEDStripConstants.RIGHT_CLIMBER_LEDS, LEDStripConstants.LEFT_CLIMBER_LEDS
        ).ignoringDisable(true);
    }

    public static Command getThreeSectionColorCommand(Supplier<Color> firstSectionColor, Supplier<Color> secondSectionColor, Supplier<Color> thirdSectionColor, LEDStrip ledStrip) {
        return new InitExecuteCommand(
                ledStrip::clearLedColors,
                () -> ledStrip.threeSectionColor(firstSectionColor.get(), secondSectionColor.get(), thirdSectionColor.get()),
                ledStrip
        ).ignoringDisable(true);
    }

    public static Command getThreeSectionColorCommand(Supplier<Color> firstSectionColor, Supplier<Color> secondSectionColor, Supplier<Color> thirdSectionColor) {
        return new InitExecuteCommand(
                () -> runForAllLeds(LEDStrip::clearLedColors),
                () -> runForAllLeds((ledStrip) -> ledStrip.threeSectionColor(firstSectionColor.get(), secondSectionColor.get(), thirdSectionColor.get())),
                LEDStripConstants.RIGHT_CLIMBER_LEDS, LEDStripConstants.LEFT_CLIMBER_LEDS
        ).ignoringDisable(true);
    }

    public static void runForAllLeds(Consumer<LEDStrip> action) {
        action.accept(LEDStripConstants.RIGHT_CLIMBER_LEDS);
        action.accept(LEDStripConstants.LEFT_CLIMBER_LEDS);
    }
}