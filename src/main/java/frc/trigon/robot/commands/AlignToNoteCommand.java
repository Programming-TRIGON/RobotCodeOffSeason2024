package frc.trigon.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.factories.GeneralCommands;
import frc.trigon.robot.constants.CameraConstants;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.objectdetectioncamera.ObjectDetectionCamera;
import frc.trigon.robot.subsystems.intake.IntakeConstants;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

public class AlignToNoteCommand extends ParallelCommandGroup {
    private static final ObjectDetectionCamera CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;
    private static final PIDController Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.0075, 0, 0) :
            new PIDController(0.0002, 0, 0);

    public AlignToNoteCommand() {
        addCommands(
                getSetCurrentLEDColorCommand(),
                GeneralCommands.getContinuousConditionalCommand(getDriveWhileAligningToNoteCommand(), GeneralCommands.duplicate(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND), this::shouldAlignToNote).asProxy(),
                new RunCommand(CAMERA::trackObject)
        );
    }

    private Command getSetCurrentLEDColorCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                LEDStripCommands.getBreatheCommand(
                        IntakeConstants.NOTE_DETECTION_CAMERA_HAS_TARGETS_BREATHING_LEDS_COLOR,
                        IntakeConstants.ALIGN_TO_NOTE_BREATHING_LEDS_AMOUNT,
                        IntakeConstants.ALIGN_TO_NOTE_BREATHING_CYCLE_TIME_SECONDS,
                        IntakeConstants.ALIGN_TO_NOTE_BREATHING_SHOULD_LOOP,
                        IntakeConstants.ALIGN_TO_NOTE_BREATHING_IS_INVERTED,
                        LEDStrip.LED_STRIPS
                ),
                LEDStripCommands.getBreatheCommand(
                        IntakeConstants.NOTE_DETECTION_CAMERA_HAS_NO_TARGETS_BREATHING_LEDS_COLOR,
                        IntakeConstants.ALIGN_TO_NOTE_BREATHING_LEDS_AMOUNT,
                        IntakeConstants.ALIGN_TO_NOTE_BREATHING_CYCLE_TIME_SECONDS,
                        IntakeConstants.ALIGN_TO_NOTE_BREATHING_SHOULD_LOOP,
                        IntakeConstants.ALIGN_TO_NOTE_BREATHING_IS_INVERTED,
                        LEDStrip.LED_STRIPS
                ),
                CAMERA::hasTargets
        ).asProxy();
    }

    private Command getDriveWhileAligningToNoteCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> fieldRelativePowersToSelfRelativeXPower(OperatorConstants.DRIVER_CONTROLLER.getLeftY(), OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                () -> -Y_PID_CONTROLLER.calculate(CAMERA.getTrackedObjectYaw().getDegrees()),
                this::getTargetAngle
        );
    }

    private double fieldRelativePowersToSelfRelativeXPower(double xPower, double yPower) {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle();
        final double xValue = CommandConstants.calculateDriveStickAxisValue(xPower);
        final double yValue = CommandConstants.calculateDriveStickAxisValue(yPower);

        return (xValue * robotHeading.getCos()) + (yValue * robotHeading.getSin());
    }

    private MirrorableRotation2d getTargetAngle() {
        final Rotation2d currentRotation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        return new MirrorableRotation2d(currentRotation.plus(CAMERA.getTrackedObjectYaw()), false);
    }

    private boolean shouldAlignToNote() {
        return CAMERA.hasTargets() && !RobotContainer.INTAKE.isEarlyNoteCollectionDetected();
    }
}