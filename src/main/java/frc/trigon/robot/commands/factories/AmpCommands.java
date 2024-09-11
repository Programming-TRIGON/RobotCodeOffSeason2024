package frc.trigon.robot.commands.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerCommands;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class AmpCommands {
    public static Command getAutonomousScoreInAmpCommand() {
        return new ParallelCommandGroup(
                getAutonomousPrepareForAmpCommand(),
                GeneralCommands.runWhenContinueTriggerPressed(getFeedToAmpCommand()),
                getPathfindToAmpCommand().andThen(GeneralCommands.duplicate(CommandConstants.FACE_AMP_COMMAND))
        );
    }

    public static Command getScoreInAmpCommand() {
        return new ParallelCommandGroup(
                getPrepareForAmpCommand(),
                GeneralCommands.runWhenContinueTriggerPressed(getFeedToAmpCommand()),
                GeneralCommands.duplicate(CommandConstants.FACE_AMP_COMMAND)
        );
    }

    /**
     * Creates a command that prepares to score in the amp for the autonomous score in amp command.
     * This command waits until we're within a certain distance from the amp to ensure that the robot doesn't hit the stage.
     * We're only waiting in the autonomous placement to allow for driver judgment in the normal placement, and so it won't interfere in a case where pose estimation isn't working in the normal placement.
     * * This command is not used for the autonomous period, but rather for the real-time autonomous amp placement.
     *
     * @return the command
     */
    private static Command getAutonomousPrepareForAmpCommand() {
        return new ParallelCommandGroup(
                GeneralCommands.runWhen(
                        AmpAlignerCommands.getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState.OPEN)
                                .alongWith(PitcherCommands.getSetTargetPitchCommand(ShootingConstants.AMP_PITCH)),
                        () -> RobotContainer.POSE_ESTIMATOR.getCurrentPose().getTranslation().getDistance(
                                FieldConstants.IN_FRONT_OF_AMP_POSE.get().getTranslation()) < FieldConstants.MINIMUM_DISTANCE_FROM_AMP_FOR_AUTONOMOUS_AMP_PREPARATION_METERS
                ),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.AMP_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    private static Command getPrepareForAmpCommand() {
        return new ParallelCommandGroup(
                AmpAlignerCommands.getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState.OPEN),
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.AMP_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.AMP_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    private static Command getFeedToAmpCommand() {
        return GeneralCommands.runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_AMP).alongWith(ShootingCommands.getVisualizeNoteShootingCommand()), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch() && RobotContainer.AMP_ALIGNER.atTargetState());
    }

    private static Command getPathfindToAmpCommand() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.IN_FRONT_OF_AMP_POSE,
                AutonomousConstants.REAL_TIME_CONSTRAINTS
        );
    }
}
