package frc.trigon.robot.commands.factories;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.AlignToNoteCommand;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.commands.VisualizeNoteShootingCommand;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerCommands;
import frc.trigon.robot.subsystems.ampaligner.AmpAlignerConstants;
import frc.trigon.robot.subsystems.climber.ClimberCommands;
import frc.trigon.robot.subsystems.climber.ClimberConstants;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;

public class GeneralCommands {
    public static boolean IS_BRAKING = true;

    public static Command getClimbCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            RobotContainer.CLIMBER.setIsClimbing(true);
                            Logger.recordOutput("IsClimbing", true);
                        }
                ),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.PREPARE_FOR_CLIMB).until(OperatorConstants.CONTINUE_TRIGGER),
                ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB)
        );
    }

    public static Command getNoteCollectionCommand() {
        return new ParallelCommandGroup(
                new AlignToNoteCommand().onlyIf(() -> CommandConstants.SHOULD_ALIGN_TO_NOTE),
                LEDStripCommands.getBreatheCommand(Color.kOrangeRed, IntakeConstants.INTAKE_INDICATION_BREATHING_LEDS_AMOUNT, IntakeConstants.INTAKE_INDICATION_BREATHING_CYCLE_TIME_SECONDS, IntakeConstants.INTAKE_INDICATION_BREATHING_SHOULD_LOOP, LEDStrip.LED_STRIPS).asProxy().onlyIf(() -> !CommandConstants.SHOULD_ALIGN_TO_NOTE),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.FINISHED_INTAKE_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND).unless(RobotContainer.INTAKE::hasNote)
        ).unless(RobotContainer.INTAKE::hasNote).alongWith(duplicate(CommandConstants.RUMBLE_COMMAND).onlyIf(RobotContainer.INTAKE::hasNote));
    }

    public static Command getHighEjectNoteCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.HIGH_EJECT_PITCH),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.EJECT)
        );
    }

    public static Command getBlockCommand() {
        return new ParallelCommandGroup(
                AmpAlignerCommands.getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState.OPEN),
                PitcherCommands.getSetTargetPitchCommand(PitcherConstants.BLOCK_PITCH)
        );
    }

    public static Command getVisualizeNoteShootingCommand() {
        return new InstantCommand(
                () -> new VisualizeNoteShootingCommand()
                        .schedule()).onlyIf(MotorSubsystem::isExtensiveLoggingEnabled);
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

    public static Command runWhen(Command command, BooleanSupplier condition, double debounceTimeSeconds) {
        return new WaitUntilCommand(condition).andThen(new WaitCommand(debounceTimeSeconds).andThen(command.onlyIf(condition)));
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

    public static Command runWhenContinueTriggerPressed(Command command) {
        return runWhen(command, OperatorConstants.CONTINUE_TRIGGER);
    }
}