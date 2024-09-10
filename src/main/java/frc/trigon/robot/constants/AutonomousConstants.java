package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);
    public static boolean shouldAlignToSpeaker = false;

    static {
        registerCommands();
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("Collect", new InstantCommand(
                () -> {
                    System.out.println("Collecting");
                    shouldAlignToSpeaker = false;
                }));
        NamedCommands.registerCommand("WarmShooting", new InstantCommand(
                () -> {
                    System.out.println("WarmingShooting");
                    shouldAlignToSpeaker = false;
                }));
        NamedCommands.registerCommand("PrepareForShooting", new InstantCommand(
                () -> {
                    System.out.println("PreparingForShooting");
                    shouldAlignToSpeaker = true;
                }));
        NamedCommands.registerCommand("FeedNote", new InstantCommand(
                () -> {
                    System.out.println("FeedingNote");
                    shouldAlignToSpeaker = false;
                }));
        NamedCommands.registerCommand("Eject", new InstantCommand(
                () -> {
                    System.out.println("Ejecting");
                    shouldAlignToSpeaker = false;
                }));
        NamedCommands.registerCommand("EjectFromShooter", new InstantCommand(
                () -> {
                    System.out.println("EjectingFromShooter");
                    shouldAlignToSpeaker = false;
                }));
    }
}
