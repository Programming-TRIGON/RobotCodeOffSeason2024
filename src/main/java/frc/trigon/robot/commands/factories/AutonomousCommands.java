package frc.trigon.robot.commands.factories;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.misc.ShootingCalculations;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import org.trigon.commands.ExecuteEndCommand;

import java.util.Optional;

/**
 * A class that contains commands that are used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    public static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();

    public static Command getAutonomousNoteCollectionCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT).unless(RobotContainer.INTAKE::hasNote),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.STOP).onlyIf(RobotContainer.INTAKE::hasNote),
                getAlignToNoteCommand()
        );
    }

    public static Command getAutonomousPrepareForSpeakerShootingCommand() {
        return new ParallelCommandGroup(
                ShootingCommands.getWarmSpeakerShotCommand(),
                getAlignToSpeakerCommand()
        );
    }

    public static Command getAutonomousShootCommand() {
        return new ParallelCommandGroup(
                getAlignToSpeakerCommand(),
                geAutonomousFeedNoteCommand()
        );
    }

    public static Command getPrepareForEjectFromShooterCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(AutonomousConstants.EJECT_FROM_SHOOTER_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(AutonomousConstants.EJECT_FROM_SHOOTER_SPEED)
        );
    }

    private static Command geAutonomousFeedNoteCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING),
                ShootingCommands.getVisualizeNoteShootingCommand()
        );
    }

    private static Command getAlignToNoteCommand() {
        return new FunctionalCommand(
                CameraConstants.NOTE_DETECTION_CAMERA::startTrackingBestObject,
                () -> getOverrideRotationCommand(Optional.of(Rotation2d.fromDegrees(CameraConstants.NOTE_DETECTION_CAMERA.getTrackedObjectYaw()))),
                (interrupted) -> getOverrideRotationCommand(Optional.empty()).onlyIf(RobotContainer.INTAKE::hasNote),
                () -> false,
                RobotContainer.INTAKE
        );
    }

    private static Command getAlignToSpeakerCommand() {
        return new ExecuteEndCommand(
                () -> getOverrideRotationCommand(Optional.of(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle().get())),
                () -> getOverrideRotationCommand(Optional.empty()).onlyIf(() -> !RobotContainer.INTAKE.hasNote())
        );
    }

    private static Command getOverrideRotationCommand(Optional<Rotation2d> rotationOverride) {
        return new InstantCommand(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> rotationOverride));
    }
}
