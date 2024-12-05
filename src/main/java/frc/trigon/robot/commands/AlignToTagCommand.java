package frc.trigon.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.factories.GeneralCommands;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.misc.ShootingCalculations;
import frc.trigon.robot.subsystems.ledstrip.LEDStrip;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.swerve.SwerveCommands;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

public class AlignToTagCommand extends ParallelCommandGroup {
    private static final PIDController PID_CONTROLLER = new PIDController(0, 0, 0);

    public AlignToTagCommand() {
        addCommands(
                getSetLEDsCommand().asProxy(),
                getAlignToTagCommand().asProxy()
        );
    }

    private Command getSetLEDsCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                LEDStripCommands.getStaticColorCommand(Color.kGreen, LEDStrip.LED_STRIPS),
                LEDStripCommands.getStaticColorCommand(Color.kRed, LEDStrip.LED_STRIPS),
                RobotContainer.POSE_ESTIMATOR::hasResult
        );
    }

    private Command getAlignToTagCommand() {
        return GeneralCommands.getContinuousConditionalCommand(
                getDriveWhileAligningToNoteCommand(),
                GeneralCommands.duplicate(CommandConstants.FIELD_RELATIVE_DRIVE_COMMAND),
                RobotContainer.POSE_ESTIMATOR::hasResult
        );
    }

    private Command getDriveWhileAligningToNoteCommand() {
        return SwerveCommands.getClosedLoopSelfRelativeDriveCommand(
                () -> fieldRelativePowersToSelfRelativeXPower(OperatorConstants.DRIVER_CONTROLLER.getLeftY(), OperatorConstants.DRIVER_CONTROLLER.getLeftX()),
                () -> -PID_CONTROLLER.calculate(RobotContainer.POSE_ESTIMATOR.getBestTagYawRelativeToRobot().getDegrees()),
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
        return ShootingCalculations.getInstance().getAngleToTarget(new Translation2d(), RobotContainer.ROBOT_SHOWCASE.getBestTagTranslationRelativeToRobot());
    }
}