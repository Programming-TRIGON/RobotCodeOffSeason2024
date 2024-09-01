package frc.trigon.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerCommands;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class AmpCommands {
    public static Command getAutonomousScoreInAmpCommand() {
        return new ParallelCommandGroup(
                getAutonomousPrepareForAmp(),
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
     * Creates a command that prepares all the components to shoot into the amp for the autonomous amp scoring command
     *
     * @return the command
     */
    public static Command getAutonomousPrepareForAmp() {
        return new ParallelCommandGroup(
                GeneralCommands.runWhen(
                        AmpAlignerCommands.getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState.OPEN)
                                .alongWith(PitcherCommands.getSetTargetPitchCommand(PitcherConstants.AMP_PITCH)),
                        () -> RobotContainer.POSE_ESTIMATOR.getCurrentPose().getTranslation().getDistance(
                                FieldConstants.IN_FRONT_OF_AMP_POSE.get().getTranslation()) < FieldConstants.MINIMUM_DISTANCE_FROM_AMP_FOR_AUTONOMOUS_AMP_PREPARATION_METERS
                ),
                ShooterCommands.getSetTargetVelocityCommand(ShooterConstants.AMP_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    public static Command getPrepareForAmpCommand() {
        return new ParallelCommandGroup(
                AmpAlignerCommands.getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState.OPEN),
                PitcherCommands.getSetTargetPitchCommand(PitcherConstants.AMP_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShooterConstants.AMP_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    public static Command getFeedToAmpCommand() {
        return GeneralCommands.runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_AMP), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch() && RobotContainer.AMP_ALIGNER.atTargetState());
    }

    public static Command getPathfindToAmpCommand() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.IN_FRONT_OF_AMP_POSE,
                AutonomousConstants.REAL_TIME_CONSTRAINTS
        );
    }
}
