package frc.trigon.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.factories.GeneralCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

import java.awt.*;

public class AlignToNoteCommand extends ParallelCommandGroup {
    private static final ObjectDetectionCamera CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;
    private static final PIDController Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.0075, 0, 0) :
            new PIDController(0, 0, 0);
    private boolean didCollect = false;
    private Rotation2d trackedNoteYaw = new Rotation2d();

    public AlignToNoteCommand() {
        addCommands(
                new InstantCommand(() -> {
                    didCollect = false;
                    didCollect = RobotContainer.INTAKE.isEarlyNoteCollectionDetected();
                }),
                getSetCurrentLEDColorCommand().asProxy(),
                GeneralCommands.getContinuousConditionalCommand(getDriveWhileAligningToNoteCommand(), GeneralCommands.duplicate(CommandConstants.SELF_RELATIVE_DRIVE_COMMAND), this::hasTarget).asProxy(),
                new RunCommand(CAMERA::trackObject),
                new RunCommand(this::updateTrackedNoteYaw)
        );
    }

    private Command getSetCurrentLEDColorCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                LEDStripCommands.getStaticColorCommand(Color.green, LEDStripConstants.LED_STRIPS),
                LEDStripCommands.getStaticColorCommand(Color.red, LEDStripConstants.LED_STRIPS),
                CAMERA::hasTargets
        );
    }

    private Command getDriveWhileAligningToNoteCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                () -> -Y_PID_CONTROLLER.calculate(trackedNoteYaw.getDegrees()),
                this::getTargetAngle
        );
    }

    private MirrorableRotation2d getTargetAngle() {
        final Rotation2d currentRotation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        return new MirrorableRotation2d(currentRotation.plus(trackedNoteYaw), false);
    }

    private void updateTrackedNoteYaw() {
        trackedNoteYaw = CAMERA.getTrackedObjectYaw();
    }

    private boolean hasTarget() {
        return CAMERA.hasTargets() && !didCollect;
    }
}