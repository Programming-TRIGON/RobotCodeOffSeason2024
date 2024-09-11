package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.factories.AutonomousCommands;
import frc.trigon.robot.commands.factories.ShootingCommands;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);
    public static final Rotation2d EJECT_FROM_SHOOTER_PITCH = Rotation2d.fromDegrees(0);
    public static final double EJECT_FROM_SHOOTER_SPEED = 0.5;

    static {
        registerCommands();
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("Collect", AutonomousCommands.getAutonomousNoteCollectionCommand());
        NamedCommands.registerCommand("WarmShooting", ShootingCommands.getWarmSpeakerShotCommand());
        NamedCommands.registerCommand("PrepareForShooting", AutonomousCommands.getAutonomousPrepareForSpeakerShootingCommand());
        NamedCommands.registerCommand("Shoot", AutonomousCommands.getAutonomousShootCommand());
        NamedCommands.registerCommand("Eject", CommandConstants.EJECT_COMMAND);
        NamedCommands.registerCommand("EjectFromShooter", AutonomousCommands.getPrepareForEjectFromShooterCommand());
    }
}
