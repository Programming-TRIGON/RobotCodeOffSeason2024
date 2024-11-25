package frc.trigon.robot.commands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.ledstrip.LEDStripCommands;
import frc.trigon.robot.subsystems.ledstrip.LEDStripConstants;
import org.trigon.utilities.mirrorable.MirrorablePose2d;

import java.util.function.Supplier;

/**
 * A command that sets the LED strips to a color based on the robot's position and orientation relative to the starting
 * pose of the selected autonomous path.
 * This is very useful for placing the robot in the correct starting position and orientation for autonomous mode before a match.
 */
public class LEDAutoSetupCommand extends SequentialCommandGroup {
    private static final double
            TOLERANCE_METERS = 0.1,
            TOLERANCE_DEGREES = 2;
    private static final int AMOUNT_OF_SECTIONS = 3;
    private final Supplier<Color>[] LEDColors = new Supplier[]{
            () -> getDesiredLEDColorFromRobotPose(this.autoStartPose.getRotation().getDegrees() - getCurrentRobotPose().getRotation().getDegrees(), TOLERANCE_DEGREES),
            () -> getDesiredLEDColorFromRobotPose(this.autoStartPose.getX() - getCurrentRobotPose().getX(), TOLERANCE_METERS),
            () -> getDesiredLEDColorFromRobotPose(this.autoStartPose.getY() - getCurrentRobotPose().getY(), TOLERANCE_METERS)
    };
    private final Supplier<String> autoName;
    private Pose2d autoStartPose;

    /**
     * Constructs a new LEDAutoSetupCommand.
     *
     * @param autoName a supplier that returns the name of the selected autonomous path
     */
    public LEDAutoSetupCommand(Supplier<String> autoName) {
        this.autoName = autoName;

        addCommands(
                getUpdateAutoStartPoseCommand(),
                LEDStripCommands.getSectionColorCommand(
                        AMOUNT_OF_SECTIONS,
                        LEDColors,
                        LEDStripConstants.RIGHT_CLIMBER_LEDS
                ).alongWith(
                        LEDStripCommands.getSectionColorCommand(
                                AMOUNT_OF_SECTIONS,
                                LEDColors,
                                LEDStripConstants.LEFT_CLIMBER_LEDS
                        )
                )
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    private Command getUpdateAutoStartPoseCommand() {
        return new InstantCommand(
                () -> this.autoStartPose = new MirrorablePose2d(PathPlannerAuto.getStaringPoseFromAutoFile(autoName.get()), true).get()
        ).ignoringDisable(true);
    }

    private Color getDesiredLEDColorFromRobotPose(double difference, double tolerance) {
        if (difference < -tolerance)
            return Color.kBlack;
        else if (difference > tolerance)
            return Color.kRed;
        return Color.kGreen;
    }

    private Pose2d getCurrentRobotPose() {
        return RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
    }
}