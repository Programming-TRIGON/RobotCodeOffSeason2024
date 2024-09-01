package frc.trigon.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CommandConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.ShootingCalculations;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
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

    public static Command getWarmSpeakerShotCommand() {
        return new ParallelCommandGroup(
                getUpdateShootingCalculationsCommand(false),
                PitcherCommands.getReachTargetPitchFromShootingCalculationsCommand(),
                ShooterCommands.getReachTargetShootingVelocityFromShootingCalculationsCommand()
        );
    }

    public static Command getFeedNoteForShootingCommand() {
        return GeneralCommands.runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch() && RobotContainer.SWERVE.atAngle(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle()));
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
        return GeneralCommands.runWhen(IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING), () -> RobotContainer.SHOOTER.atTargetVelocity() && RobotContainer.PITCHER.atTargetPitch());
    }

    private static Command getUpdateShootingCalculationsCommand(boolean isDelivery) {
        return new RunCommand(isDelivery ? SHOOTING_CALCULATIONS::updateCalculationsForDelivery : SHOOTING_CALCULATIONS::updateCalculationsForSpeakerShot);
    }
}
