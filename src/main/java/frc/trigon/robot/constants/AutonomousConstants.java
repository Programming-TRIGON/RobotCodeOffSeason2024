package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import frc.trigon.robot.commands.factories.AutonomousCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);
    public static final double AUTONOMOUS_FEEDING_TIME_SECONDS = 0.7;

    static {
        registerCommands();
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("Collect", AutonomousCommands.getNoteCollectionCommand());
        NamedCommands.registerCommand("AlignToNote", AutonomousCommands.getAlignToNoteCommand());
        NamedCommands.registerCommand("StopAligningToNote", AutonomousCommands.getStopAligningToNoteCommand());
        NamedCommands.registerCommand("PrepareForShooting", AutonomousCommands.getPrepareShooterForSpeakerShotCommand());
        NamedCommands.registerCommand("PreparePitch", AutonomousCommands.getPreparePitchForSpeakerShotCommand());
        NamedCommands.registerCommand("PrepareForShootingCloseShot", AutonomousCommands.getPrepareShooterCommand(ShootingConstants.CLOSE_SHOT_VELOCITY_ROTATIONS_PER_SECOND));
        NamedCommands.registerCommand("PreparePitchForCloseShot", AutonomousCommands.getPreparePitchCommand(ShootingConstants.CLOSE_SHOT_PITCH));
        NamedCommands.registerCommand("StopShooting", ShooterCommands.getStopCommand());
        NamedCommands.registerCommand("FeedNote", AutonomousCommands.getFeedNoteCommand());
        NamedCommands.registerCommand("PreparePitchForEjectFromShooter", AutonomousCommands.getPreparePitchCommand(ShootingConstants.EJECT_FROM_SHOOTER_PITCH));
        NamedCommands.registerCommand("PrepareShootingForEjectFromShooter", AutonomousCommands.getPrepareShooterCommand(ShootingConstants.EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND));
        NamedCommands.registerCommand("PreparePitchForCloseEjectFromShooter", AutonomousCommands.getPreparePitchCommand(ShootingConstants.CLOSE_EJECT_FROM_SHOOTER_PITCH));
        NamedCommands.registerCommand("PrepareShootingForCloseEjectFromShooter", AutonomousCommands.getPrepareShooterCommand(ShootingConstants.CLOSE_EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND));
    }
}