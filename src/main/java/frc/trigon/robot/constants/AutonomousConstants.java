package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.factories.AutonomousCommands;
import frc.trigon.robot.commands.factories.ShootingCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

    static {
        registerCommands();
        PPHolonomicDriveController.setRotationTargetOverride(AutonomousCommands::getRotationOverride);
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("Collect", AutonomousCommands.getNoteCollectionCommand());
        NamedCommands.registerCommand("AlignToNote", AutonomousCommands.getAlignToNoteCommand());
        NamedCommands.registerCommand("AlignToSpeaker", AutonomousCommands.getAlignToSpeakerCommand());
        NamedCommands.registerCommand("PrepareForShooting", ShootingCommands.getWarmSpeakerShotCommand());
        NamedCommands.registerCommand("StopShooting", ShooterCommands.getStopCommand());
        NamedCommands.registerCommand("FeedNote", AutonomousCommands.getFeedNoteCommand());
        NamedCommands.registerCommand("PrepareForCloseShot", ShootingCommands.getPrepareCloseSpeakerShotCommand());
        NamedCommands.registerCommand("Eject", CommandConstants.EJECT_COMMAND);
        NamedCommands.registerCommand("EjectFromShooter", AutonomousCommands.getPrepareForShooterEjectionCommand());
    }
}