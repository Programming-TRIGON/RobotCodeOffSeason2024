package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.commands.factories.AutonomousCommands;
import frc.trigon.robot.commands.factories.ShootingCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

    static {
        registerCommands();
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("Collect", AutonomousCommands.getNoteCollectionCommand());
        NamedCommands.registerCommand("AlignToNote", AutonomousCommands.getAlignToNoteCommand());
        NamedCommands.registerCommand("StopAligningToNote", AutonomousCommands.getStopAligningToNoteCommand());
        NamedCommands.registerCommand("PrepareForShooting", AutonomousCommands.getPrepareShootingCommand());
        NamedCommands.registerCommand("PreparePitch", AutonomousCommands.getPreparePitchCommand());
        NamedCommands.registerCommand("PrepareForShootingCloseShot", AutonomousCommands.getPrepareShootingCommand(ShootingConstants.CLOSE_SHOT_VELOCITY_ROTATIONS_PER_SECOND));
        NamedCommands.registerCommand("PreparePitchForCloseShot", AutonomousCommands.getPreparePitchCommand(ShootingConstants.CLOSE_SHOT_PITCH));
        NamedCommands.registerCommand("StopShooting", ShooterCommands.getStopCommand());
        NamedCommands.registerCommand("FeedNote", AutonomousCommands.getFeedNoteCommand());
        NamedCommands.registerCommand("PreparePitchForEjectFromShooter", AutonomousCommands.getPreparePitchCommand(ShootingConstants.EJECT_FROM_SHOOTER_PITCH));
        NamedCommands.registerCommand("PrepareShootingForEjectFromShooter", AutonomousCommands.getPrepareShootingCommand(ShootingConstants.EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND));
        NamedCommands.registerCommand("PreparePitchForCloseEjectFromShooter", AutonomousCommands.getPreparePitchCommand(ShootingConstants.CLOSE_EJECT_FROM_SHOOTER_PITCH));
        NamedCommands.registerCommand("PrepareShootingForCloseEjectFromShooter", AutonomousCommands.getPrepareShootingCommand(ShootingConstants.CLOSE_EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND));
    }
}