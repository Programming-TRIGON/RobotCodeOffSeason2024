package frc.trigon.robot.constants;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.factories.AutonomousCommands;
import frc.trigon.robot.commands.factories.ShootingCommands;

public class AutonomousConstants {
    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);

    static {
        registerCommands();
    }

    private static void registerCommands() {
        NamedCommands.registerCommand("Collect", AutonomousCommands.getNoteCollectionCommand());
        NamedCommands.registerCommand("AlignToNote", AutonomousCommands.getAlignToNoteCommand());
        NamedCommands.registerCommand("AlignToSpeaker", AutonomousCommands.getAlignToSpeakerCommand());
        NamedCommands.registerCommand("PrepareForShooting", ShootingCommands.getWarmSpeakerShotCommand());
        NamedCommands.registerCommand("FeedNote", AutonomousCommands.getFeedNoteCommand());
        NamedCommands.registerCommand("CloseShot", ShootingCommands.getPrepareCloseSpeakerShotCommand());
        NamedCommands.registerCommand("Eject", CommandConstants.EJECT_COMMAND);
        NamedCommands.registerCommand("EjectFromShooter", AutonomousCommands.getPrepareForShooterEjectionCommand());
    }
}
