package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.misc.ShootingCalculations;
import org.littletonrobotics.junction.Logger;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;

/**
 * A command to visualize note shooting.
 * This command will get the physical information from subsystems when we begin the shot, and calculate the note's position at each timestamp using physics.
 */
public class VisualizeNoteShootingCommand extends Command {
    private static final ShootingCalculations SHOOTING_CALCULATIONS = ShootingCalculations.getInstance();
    private double startingTimeSeconds;
    private Translation3d fieldRelativeNoteExitPointTranslation;
    private Rotation2d startingPitch;
    private Translation2d initialXYVelocity;
    private double initialZVelocity;
    private double noteZ;

    @Override
    public void initialize() {
        startingTimeSeconds = Timer.getFPGATimestamp();
        startingPitch = RobotContainer.PITCHER.getCurrentPitch();
        final Pose2d currentRobotPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose();
        final Rotation2d currentRobotAngle = currentRobotPose.getRotation();
        final Pose3d fieldRelativeNoteExitPointPose = SHOOTING_CALCULATIONS.calculateShooterNoteExitPointFieldRelativePose(startingPitch, currentRobotPose.getTranslation(), new MirrorableRotation2d(currentRobotAngle, false));
        final double startingTangentialVelocity = SHOOTING_CALCULATIONS.angularVelocityToTangentialVelocity(RobotContainer.SHOOTER.getCurrentRightMotorVelocityRotationsPerSecond());
        fieldRelativeNoteExitPointTranslation = fieldRelativeNoteExitPointPose.getTranslation();
        initialXYVelocity = calculateInitialXYVelocityWithRobotVelocity(startingPitch, startingTangentialVelocity, currentRobotAngle);
        initialZVelocity = startingPitch.getSin() * startingTangentialVelocity;
    }

    @Override
    public void execute() {
        final double t = Timer.getFPGATimestamp() - startingTimeSeconds;

        final double noteXPosition = calculateNoteXPosition(t);
        final double noteYPosition = calculateNoteYPosition(t);
        final double noteZPosition = calculateNoteZPosition(t);

        final Transform3d noteTransform = new Transform3d(noteXPosition, noteYPosition, noteZPosition, new Rotation3d(0, -startingPitch.getRadians(), 0));
        final Pose3d notePose = new Pose3d(fieldRelativeNoteExitPointTranslation, new Rotation3d()).plus(noteTransform);
        noteZ = notePose.getTranslation().getZ();
        Logger.recordOutput("Poses/GamePieces/ShotNotePose", notePose);
    }

    @Override
    public boolean isFinished() {
        return noteZ < 0;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Poses/GamePieces/ShotNotePose", new Pose3d(0, 0, 50, new Rotation3d()));
    }

    private Translation2d calculateInitialXYVelocityWithRobotVelocity(Rotation2d startingPitch, double startingTangentialVelocity, Rotation2d currentRobotAngle) {
        final Translation2d robotFieldRelativeVelocity = SHOOTING_CALCULATIONS.getRobotFieldRelativeVelocity().toTranslation2d();
        final double noteXYVelocityRelativeToRobot = startingPitch.getCos() * startingTangentialVelocity;
        return robotFieldRelativeVelocity.plus(new Translation2d(noteXYVelocityRelativeToRobot, currentRobotAngle.minus(new Rotation2d(Math.PI))));
    }

    private double calculateNoteXPosition(double t) {
        return initialXYVelocity.getX() * t;
    }

    private double calculateNoteYPosition(double t) {
        return initialXYVelocity.getY() * t;
    }

    private double calculateNoteZPosition(double t) {
        return (initialZVelocity * t) + (-0.5 * ShootingConstants.G_FORCE * t * t);
    }
}
