package frc.trigon.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.AutonomousConstants;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerCommands;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import frc.trigon.robot.utilities.ShootingCalculations;

import java.util.function.BooleanSupplier;

public class Commands {
    public static boolean IS_BRAKING = true;
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();

    public static Command withoutRequirements(Command command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished
        );
    }

    /**
     * @return a command that toggles between the SWERVE's default command, from field relative to self relative
     */
    public static Command getToggleFieldAndSelfRelativeDriveCommand() {
        return new InstantCommand(() -> {
            if (RobotContainer.SWERVE.getDefaultCommand().equals(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND))
                RobotContainer.SWERVE.setDefaultCommand(CommandConstants.SELF_RELATIVE_DRIVE_COMMAND);
            else
                RobotContainer.SWERVE.setDefaultCommand(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND);

            RobotContainer.SWERVE.getDefaultCommand().schedule();
        });
    }

    public static Command getToggleBrakeCommand() {
        return new InstantCommand(() -> {
            IS_BRAKING = !IS_BRAKING;
            MotorSubsystem.setAllSubsystemsBrakeAsync(IS_BRAKING);

            if (IS_BRAKING)
                CommandConstants.STATIC_WHITE_LED_COLOR_COMMAND.cancel();
            else
                CommandConstants.STATIC_WHITE_LED_COLOR_COMMAND.schedule();
        }).ignoringDisable(true);
    }

    public static Command getDelayedCommand(double delaySeconds, Runnable toRun) {
        return new WaitCommand(delaySeconds).andThen(toRun).ignoringDisable(true);
    }

    public static Command getContinuousConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
        return new ConditionalCommand(
                onTrue.onlyWhile(condition),
                onFalse.onlyWhile(() -> !condition.getAsBoolean()),
                condition
        ).repeatedly();
    }

    public static Command runWhen(Command command, BooleanSupplier condition) {
        return new WaitUntilCommand(condition).andThen(command);
    }

    public static Command duplicate(Command command) {
        return new FunctionalCommand(
                command::initialize,
                command::execute,
                command::end,
                command::isFinished,
                command.getRequirements().toArray(Subsystem[]::new)
        );
    }

    /**
     * Creates a command that sets the shooting mechanism to shoot at the given target
     *
     * @param isDelivery if the robot is shooting the note for a delivery or for a speaker shot
     * @return the command
     */
    public static Command getShootAtShootingTargetCommand(boolean isDelivery) {
        return new ParallelCommandGroup(
                getPrepareForShootingCommand(isDelivery),
                getFeedNoteForShootingCommand()
        );
    }

    public static Command getPrepareForShootingCommand(boolean isDelivery) {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(isDelivery),
                PitcherCommands.getReachTargetPitchFromShootingCalculationsCommand(),
                ShooterCommands.getReachTargetShootingVelocityFromShootingCalculationsCommand(),
                SwerveCommands.getClosedLoopFieldRelativeDriveCommand(
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                        () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                        () -> SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle()
                )
        );
    }

    public static Command getWarmForSpeakerShotCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(false),
                PitcherCommands.getReachTargetPitchFromShootingCalculationsCommand(),
                ShooterCommands.getReachTargetShootingVelocityFromShootingCalculationsCommand()
        );
    }

    public static Command getFeedNoteForShootingCommand() {
        return runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch() && RobotContainer.SWERVE.atAngle(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle()));
    }

    public static Command getCloseSpeakerShotCommand() {
        return new ParallelCommandGroup(
                getPrepareCloseSpeakerShotCommand(),
                getFeedNoteForCloseSpeakerShotCommand()
        );
    }

    public static Command getPrepareCloseSpeakerShotCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(PitcherConstants.CLOSE_SHOT_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShooterConstants.CLOSE_SHOT_VELOCITY_ROTATIONS_PER_SECOND, ShooterConstants.CLOSE_SHOT_VELOCITY_ROTATIONS_PER_SECOND * ShooterConstants.RIGHT_MOTOR_TO_LEFT_MOTOR_RATIO)
        );
    }

    public static Command getFeedNoteForCloseSpeakerShotCommand() {
        return runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch());
    }

    public static Command getAutonomousScoreInAmpCommand() {
        return new ParallelCommandGroup(
                getPrepareForAmpCommand(),
                runWhenContinueTriggerPressed(getFeedToAmpCommand()),
                getPathfindToAmpCommand().andThen(duplicate(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND))
        );
    }

    public static Command getScoreInAmpCommand() {
        return new ParallelCommandGroup(
                getPrepareForAmpCommand(),
                runWhenContinueTriggerPressed(getFeedToAmpCommand()),
                duplicate(CommandConstants.FACE_AMP_COMMAND)
        );
    }

    public static Command getPrepareForAmpCommand() {
        return new ParallelCommandGroup(
                AmpAlignerCommands.getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState.OPEN),
                PitcherCommands.getSetTargetPitchCommand(PitcherConstants.AMP_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShooterConstants.AMP_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND, ShooterConstants.AMP_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    public static Command getFeedToAmpCommand() {
        return runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_AMP), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch() && RobotContainer.AMP_ALIGNER.atTargetState());
    }

    public static Command getPathfindToAmpCommand() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.IN_FRONT_OF_AMP_POSE,
                AutonomousConstants.REAL_TIME_CONSTRAINTS
        );
    }

    /**
     * If the pitcher closes before the amp aligner is closed, the amp aligner hits the amp.
     * This command ensures that the pitcher closes after the amp is closed enough.
     *
     * @return the default pitcher command
     */
    public static Command getDefaultPitcherCommand() {
        return runWhen(PitcherCommands.getSetTargetPitchCommand(PitcherConstants.DEFAULT_PITCH), RobotContainer.AMP_ALIGNER::isReadyForDefaultPitcherMovement);
    }

    private static Command runWhenContinueTriggerPressed(Command command) {
        return runWhen(command, OperatorConstants.CONTINUE_TRIGGER);
    }

    private static Command getUpdateShootingCalculationsCommand(boolean isDelivery) {
        return new RunCommand(isDelivery ? SHOOTING_CALCULATIONS::updateCalculationsForDelivery : SHOOTING_CALCULATIONS::updateCalculationsForSpeakerShot);
    }
}
