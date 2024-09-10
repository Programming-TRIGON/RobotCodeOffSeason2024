package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);
    public static boolean shouldUpdateRobotAngle = false;

    static {
        registerCommands();
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("Collect", new InstantCommand(
                () -> {
                    System.out.println("Collecting");
                    shouldUpdateRobotAngle = false;
                }));
        NamedCommands.registerCommand("WarmShooting", new InstantCommand(
                () -> {
                    System.out.println("WarmingShooting");
                    shouldUpdateRobotAngle = false;
                }));
        NamedCommands.registerCommand("PrepareForShooting", new InstantCommand(
                () -> {
                    System.out.println("PreparingForShooting");
                    shouldUpdateRobotAngle = true;
                }));
        NamedCommands.registerCommand("FeedNote", new InstantCommand(
                () -> {
                    System.out.println("FeedingNote");
                    shouldUpdateRobotAngle = true;
                }));
        NamedCommands.registerCommand("Eject", new InstantCommand(
                () -> {
                    System.out.println("Ejecting");
                    shouldUpdateRobotAngle = false;
                }));
        NamedCommands.registerCommand("EjectFromShooter", new InstantCommand(
                () -> {
                    System.out.println("EjectingFromShooter");
                    shouldUpdateRobotAngle = false;
                }));
    }
}
