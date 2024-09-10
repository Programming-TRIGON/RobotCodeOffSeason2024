package frc.trigon.robot.commands.factories;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.misc.ShootingCalculations;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;

import java.util.Optional;

public class AutonomousCommands {
    public static ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();

    public static Command getAutonomousNoteCollectionCommand() {
        return new ParallelCommandGroup(
                GeneralCommands.getNoteCollectionCommand(),
                getAlignToNoteCommand()
        );
    }

    public static Command getAutonomousPrepareForShootingCommand() {
        return new ParallelCommandGroup(
                ShootingCommands.getWarmSpeakerShotCommand(),
                getAlignToSpeakerCommand()
        );
    }

    public static Command getEjectFromShooterCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(AutonomousConstants.EJECT_FROM_SHOOTER_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(AutonomousConstants.EJECT_FROM_SHOOTER_SPEED)
        );
    }

    private static Command getAlignToNoteCommand() {
        return new SequentialCommandGroup(
                getOverrideRotationCommand(Optional.of(Rotation2d.fromDegrees(CameraConstants.NOTE_DETECTION_CAMERA.getTrackedObjectYaw()))),
                getOverrideRotationCommand(Optional.empty()).onlyIf(RobotContainer.INTAKE::hasNote)
        );
    }

    private static Command getAlignToSpeakerCommand() {
        return new SequentialCommandGroup(
                getOverrideRotationCommand(Optional.of(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle().get())),
                getOverrideRotationCommand(Optional.empty()).onlyIf(() -> !RobotContainer.INTAKE.hasNote())
        );
    }

    private static Command getOverrideRotationCommand(Optional<Rotation2d> rotationOverride) {
        return new InstantCommand(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> rotationOverride));
    }
}
