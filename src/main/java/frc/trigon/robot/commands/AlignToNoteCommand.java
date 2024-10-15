package frc.trigon.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
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
import org.trigon.hardware.misc.XboxController;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

public class AlignToNoteCommand extends ParallelCommandGroup {
    private static final ObjectDetectionCamera CAMERA = CameraConstants.NOTE_DETECTION_CAMERA;
    private static final PIDController Y_PID_CONTROLLER = RobotHardwareStats.isSimulation() ?
            new PIDController(0.0075, 0, 0) :
            new PIDController(0.0002, 0, 0);
    private double lastTimeCameraHadObjectsSeconds = 0;

    public AlignToNoteCommand() {
        addCommands(
                getSetCurrentLEDColorCommand().asProxy(),
                GeneralCommands.getContinuousConditionalCommand(getDriveWhileAligningToNoteCommand(), GeneralCommands.duplicate(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND), this::shouldAlignToNote).asProxy(),
                new RunCommand(CAMERA::trackObject)
        );
    }

    private Command getSetCurrentLEDColorCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                LEDStripCommands.getStaticColorCommand(Color.kGreen, LEDStrip.LED_STRIPS),
                LEDStripCommands.getStaticColorCommand(Color.kRed, LEDStrip.LED_STRIPS),
                CAMERA::hasTargets
        );
    }

    private Command getDriveWhileAligningToNoteCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> CommandConstants.calculateDriveStickAxisValue(getScaledJoystickValue()),
                () -> -Y_PID_CONTROLLER.calculate(CAMERA.getTrackedObjectYaw().getDegrees()),
                this::getTargetAngle
        );
    }

    private MirrorableRotation2d getTargetAngle() {
        final Rotation2d currentRotation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation();
        return new MirrorableRotation2d(currentRotation.plus(CAMERA.getTrackedObjectYaw()), false);
    }

    private boolean shouldAlignToNote() {
        return CAMERA.hasTargets() && !RobotContainer.INTAKE.isEarlyNoteCollectionDetected();
    }

    private double getScaledJoystickValue() {
        final XboxController controller = OperatorConstants.DRIVER_CONTROLLER;
        final Rotation2d robotRelativeToFieldRelativeRotation = Rotation2d.fromDegrees(90);
        final Rotation2d robotHeading = RobotContainer.SWERVE.getDriveRelativeAngle().plus(robotRelativeToFieldRelativeRotation);

        return controller.getLeftX() * -robotHeading.getCos() - controller.getLeftY() * -robotHeading.getSin();
    }
}