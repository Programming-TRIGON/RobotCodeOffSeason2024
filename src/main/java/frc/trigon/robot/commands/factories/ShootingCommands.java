package frc.trigon.robot.commands.factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.misc.ShootingCalculations;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;

public class ShootingCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();

    /**
     * Creates a command that adjusts the shooting mechanism to aim at a target (either delivery target or speaker target), and feeds the note once the shooting mechanism is ready to shoot (all setpoints were reached).
     *
     * @param isDelivery if the robot is shooting the note for a delivery or for a speaker shot
     * @return the command
     */
    public static Command getShootAtShootingTargetCommand(boolean isDelivery) {
        return new ParallelRaceGroup(
                getPrepareForShootingCommand(isDelivery),
                getFeedNoteForShootingCommand()
        );
    }

    public static Command getWarmSpeakerShotCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(false),
                PitcherCommands.getReachTargetPitchFromShootingCalculationsCommand(),
                ShooterCommands.getReachTargetShootingVelocityFromShootingCalculationsCommand()
        );
    }

    public static Command getCloseSpeakerShotCommand() {
        return new ParallelCommandGroup(
                getPrepareCloseSpeakerShotCommand(),
                GeneralCommands.runWhenContinueTriggerPressed(getFeedNoteWhenPitcherAndShooterReadyCommand())
        );
    }

    public static Command getManualLowDeliveryCommand() {
        return new ParallelCommandGroup(
                getPrepareManualLowDeliveryCommand(),
                getFeedNoteWhenPitcherAndShooterReadyCommand()
        );
    }

    public static Command getPrepareCloseSpeakerShotCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.CLOSE_SHOT_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.CLOSE_SHOT_VELOCITY_ROTATIONS_PER_SECOND, ShootingConstants.CLOSE_SHOT_VELOCITY_ROTATIONS_PER_SECOND * ShooterConstants.RIGHT_MOTOR_TO_LEFT_MOTOR_RATIO)
        );
    }

    static Command getUpdateShootingCalculationsCommand(boolean isDelivery) {
        return new RunCommand(isDelivery ? SHOOTING_CALCULATIONS::updateCalculationsForDelivery : SHOOTING_CALCULATIONS::updateCalculationsForSpeakerShot);
    }

    private static Command getPrepareForShootingCommand(boolean isDelivery) {
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

    private static Command getPrepareManualLowDeliveryCommand() {
        return new ParallelCommandGroup(
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.MANUAL_LOW_DELIVERY_SHOOTING_VELOCITY_ROTATIONS_PER_SECOND),
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.MANUAL_LOW_DELIVERY_PITCH)
        );
    }

    private static Command getFeedNoteForShootingCommand() {
        return GeneralCommands.runWhen(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING)
                        .alongWith(GeneralCommands.getVisualizeNoteShootingCommand()),
                () -> RobotContainer.SHOOTER.atTargetVelocity() &&
                        RobotContainer.PITCHER.atTargetPitch() &&
                        RobotContainer.SWERVE.atAngle(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle())
        );
    }

    private static Command getFeedNoteWhenPitcherAndShooterReadyCommand() {
        return GeneralCommands.runWhen(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING).alongWith(GeneralCommands.getVisualizeNoteShootingCommand()),
                () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch());
    }
}