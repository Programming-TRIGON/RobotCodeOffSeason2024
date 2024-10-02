package frc.trigon.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.factories.GeneralCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

public class AlignToNoteCommand extends ParallelCommandGroup {
    private static final ObjectDetectionCamera CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;
    private static final PIDController Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.0075, 0, 0) :
            new PIDController(0, 0, 0);
    private boolean didCollect = false;
    private boolean wasVisible = false;
    private double trackedNoteYaw = 0;

    public AlignToNoteCommand() {
        addCommands(
                new InstantCommand(() -> {
                    didCollect = false;
                    wasVisible = false;
                    didCollect = RobotContainer.INTAKE.isEarlyNoteCollectionDetected();
                }),
                getCurrentLEDColorCommand().asProxy(),
                GeneralCommands.getContinuousConditionalCommand(getDriveWhileAligningToNoteCommand(), GeneralCommands.duplicate(CommandConstants.SELF_RELATIVE_DRIVE_COMMAND), this::hasTarget).asProxy(),
                new RunCommand(this::trackObject)
        );
    }

    private void trackObject() {
        if (hasTarget() && !wasVisible) {
            wasVisible = true;
            CAMERA.startTrackingBestObject();
            trackedNoteYaw = CAMERA.getTrackedObjectYaw();
            return;
        }

        if (!hasTarget()) {
            wasVisible = false;
            return;
        }

        if (hasTarget())
            trackedNoteYaw = CAMERA.getTrackedObjectYaw();
    }

    private Command getCurrentLEDColorCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                LEDStripCommands.getStaticColorCommand(Color.kGreen, LEDStrip.LED_STRIPS.toArray(LEDStrip[]::new)),
                LEDStripCommands.getStaticColorCommand(Color.kRed, LEDStrip.LED_STRIPS.toArray(LEDStrip[]::new)),
                CAMERA::hasTargets
        );
    }

    private Command getDriveWhileAligningToNoteCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(OperatorConstants.DRIVER_CONTROLLER.getLeftY()),
                () -> -Y_PID_CONTROLLER.calculate(trackedNoteYaw),
                this::getTargetAngle
        );
    }

    private MirrorableRotation2d getTargetAngle() {
        final Rotation2d currentRotation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        return new MirrorableRotation2d(currentRotation.plus(Rotation2d.fromDegrees(trackedNoteYaw)), false);
    }

    private boolean hasTarget() {
        return CAMERA.hasTargets() && !didCollect;
    }
}