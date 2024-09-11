package frc.trigon.robot.commands.factories;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.misc.ShootingCalculations;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.subsystems.intake.IntakeCommands;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherCommands;
import frc.trigon.robot.subsystems.shooter.ShooterCommands;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.utilities.mirrorable.MirrorablePose2d;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * A class that contains commands that are used during the 15-second autonomous period at the start of each match.
 */
public class AutonomousCommands {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private static final ObjectDetectionCamera NOTE_DETECTION_CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;

    public static Command getResetPoseToAutoPoseCommand(Supplier<String> pathName) {
        return new InstantCommand(
                () -> {
                    if (DriverStation.isEnabled())
                        return;
                    final Pose2d autoStartPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathName.get());
                    RobotContainer.POSE_ESTIMATOR.resetPose(new MirrorablePose2d(autoStartPose, true).get());
                }
        ).ignoringDisable(true);
    }

    public static Command getAutonomousNoteCollectionCommand() {
        return new SequentialCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECT).unless(RobotContainer.INTAKE::hasNote),
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.STOP).onlyIf(RobotContainer.INTAKE::hasNote),
                getAlignToNoteCommand()
        );
    }

    public static Command getPrepareForShooterEjectionCommand() {
        return new ParallelCommandGroup(
                PitcherCommands.getSetTargetPitchCommand(ShootingConstants.EJECT_FROM_SHOOTER_PITCH),
                ShooterCommands.getSetTargetVelocityCommand(ShootingConstants.EJECT_FROM_SHOOTER_VELOCITY_ROTATIONS_PER_SECOND)
        );
    }

    public static Command geAutonomousFeedNoteCommand() {
        return new ParallelCommandGroup(
                IntakeCommands.getSetTargetStateCommand(IntakeConstants.IntakeState.FEED_SHOOTING),
                ShootingCommands.getVisualizeNoteShootingCommand()
        );
    }

    public static Command getAlignToSpeakerCommand() {
        return new ExecuteEndCommand(
                () -> overrideRotation(Optional.of(SHOOTING_CALCULATIONS.getTargetShootingState().targetRobotAngle().get())),
                () -> {
                    if (!RobotContainer.INTAKE.hasNote())
                        overrideRotation(Optional.empty());
                }
        );
    }

    private static Command getAlignToNoteCommand() {
        NOTE_DETECTION_CAMERA.startTrackingObject();
        return new ExecuteEndCommand(
                () -> overrideRotation(Optional.of(Rotation2d.fromDegrees(NOTE_DETECTION_CAMERA.getTrackedObjectYaw()))),
                () -> {
                    if (RobotContainer.INTAKE.hasNote())
                        overrideRotation(Optional.empty());
                }
        );
    }

    private static void overrideRotation(Optional<Rotation2d> rotationOverride) {
        PPHolonomicDriveController.setRotationTargetOverride(() -> rotationOverride);
    }
}