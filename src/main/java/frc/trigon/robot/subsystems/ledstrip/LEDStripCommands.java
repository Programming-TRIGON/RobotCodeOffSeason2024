package frc.trigon.robot.subsystems.ledstrip;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import org.trigon.commands.ExecuteEndCommand;

import java.util.function.Supplier;

public class LEDStripCommands {
    public static Command getStaticColorCommand(Color color) {
        return new StartEndCommand(
                () -> LEDStripConstants.LED_STRIP.staticColor(color),
                () -> {
                }
        ).ignoringDisable(true);
    }

    public static Command getFlashCommand(Color color) {
        return new SequentialCommandGroup(
                getStaticColorCommand(color).withTimeout(LEDStripConstants.FLASHING_TIME),
                getClearLEDsCommand()
        ).ignoringDisable(true);
    }

    public static Command getBlinkingCommand(Color color) {
        return new RunCommand(() -> LEDStripConstants.LED_STRIP.blink(color)).withTimeout(LEDStripConstants.BLINKING_INTERVAL).ignoringDisable(true);
    }

    public static Command getRainbowCommand() {
        return new RunCommand(LEDStripConstants.LED_STRIP::rainbow).ignoringDisable(true);
    }

    public static Command getThreeSectionColorCommand(Supplier<Color> firstSectionColor, Supplier<Color> secondSectionColor, Supplier<Color> thirdSectionColor) {
        return new ExecuteEndCommand(
                LEDStripConstants.LED_STRIP::clearLedColors,
                () -> LEDStripConstants.LED_STRIP.threeSectionColor(firstSectionColor.get(), secondSectionColor.get(), thirdSectionColor.get())

        ).ignoringDisable(true);
    }

    private static Command getClearLEDsCommand() {
        return new StartEndCommand(
                LEDStripConstants.LED_STRIP::clearLedColors,
                () -> {
                }
        ).ignoringDisable(true);
    }
}