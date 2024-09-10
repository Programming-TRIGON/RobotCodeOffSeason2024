package frc.trigon.robot.commands.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

public class AutonomousCommands {
    public static boolean shouldAlignToSpeaker = false;

    public static Command getAutonomousPrepareForShootingCommand() {
        return new ParallelCommandGroup(
                ShootingCommands.getWarmSpeakerShotCommand(),
                new InstantCommand(() -> shouldAlignToSpeaker = true)
        );
    }

    public static Command getAutonomousShootCommand() {
        return new SequentialCommandGroup(
                CommandConstants.SHOOT_AT_SPEAKER_COMMAND,
                new InstantCommand(() -> shouldAlignToSpeaker = false)
        );
    }

    public static Command getEjectFromShooterCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(AutonomousConstants.EJECT_FROM_SHOOTER_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(AutonomousConstants.EJECT_FROM_SHOOTER_SPEED)
        );
    }
}
