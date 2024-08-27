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
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
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
     * @param isDelivery if the robot is shooting the note for a delivery or for a shot
     * @return a command that sets the robot to shoot at the given target
     */
    public static Command getShootAtShootingTargetCommand(boolean isDelivery) {
        return new ParallelCommandGroup(
                getPrepareShooterCommand(isDelivery),
                getFeedNoteForShooterCommand()
        );
    }

    public static Command getPrepareShooterCommand(boolean isDelivery) {
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

    public static Command getShootCloseShotCommand() {
        return new ParallelCommandGroup(
                getPrepareShooterForCloseShotCommand(),
                getFeedNoteForCloseShotCommand()
        );
    }

    public static Command getPrepareShooterForCloseShotCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetPitchForCloseShot(),
                ShooterCommands.getShootCloseShotCommand()
        );
    }

    public static Command getFeedNoteForCloseShotCommand() {
        return runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch());
    }

    public static Command getFeedNoteForShooterCommand() {
        return runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch() && RobotContainer.SWERVE.atAngle(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle()));
    }

    public static Command getAutonomousScoreInAmpCommand() {
        return new ParallelCommandGroup(
                getScoreInAmpCommand(),
                getPathfindToAmpCommand()
        );
    }

    public static Command getScoreInAmpCommand() {
        return new ParallelCommandGroup(
                getPrepareForAmpCommand(),
                runWhenContinueTriggerPressed(getFeedForAmpCommand())
        );
    }

    public static Command getPrepareForAmpCommand() {
        return new ParallelCommandGroup(
                AmpAlignerCommands.getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState.OPEN),
                PitcherCommands.getPitchToAmpCommand(),
                ShooterCommands.getShootToAmpCommand()
        );
    }

    public static Command getPathfindToAmpCommand() {
        return SwerveCommands.getDriveToPoseCommand(
                () -> FieldConstants.IN_FRONT_OF_AMP_POSE,
                AutonomousConstants.REAL_TIME_CONSTRAINTS
        );
    }

    public static Command getFeedForAmpCommand() {
        return runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_AMP), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch() && RobotContainer.AMP_ALIGNER.atTargetState());
    }

    private static Command runWhenContinueTriggerPressed(Command command) {
        return runWhen(command, OperatorConstants.CONTINUE_TRIGGER);
    }

    private static Command getUpdateShootingCalculationsCommand(boolean isDelivery) {
        return new RunCommand(isDelivery ? SHOOTING_CALCULATIONS::updateCalculationsForDelivery : SHOOTING_CALCULATIONS::updateCalculationsForSpeakerShot);
    }
}
