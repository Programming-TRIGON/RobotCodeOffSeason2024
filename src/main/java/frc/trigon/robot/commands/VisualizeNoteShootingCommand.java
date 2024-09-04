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
        configureStartingStats();
    }

    @Override
    public void execute() {
        final double timeDifference = Timer.getFPGATimestamp() - startingTimeSeconds;

        final Transform3d noteTransform = calculateNoteTransform(timeDifference);
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

    private Transform3d calculateNoteTransform(double timeDifference) {
        return new Transform3d(calculateNoteXDifference(timeDifference), calculateNoteYDifference(timeDifference), calculateNoteZDifference(timeDifference), new Rotation3d(0, -startingPitch.getRadians(), 0));
    }

    private double calculateNoteXDifference(double t) {
        return initialXYVelocity.getX() * t;
    }

    private double calculateNoteYDifference(double t) {
        return initialXYVelocity.getY() * t;
    }

    private void configureStartingStats() {
        startingTimeSeconds = Timer.getFPGATimestamp();

        final Pose2d currentRobotPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose();
        final Rotation2d currentRobotAngle = currentRobotPose.getRotation();
        final double startingTangentialVelocity = getStartingTangentialVelocity();

        startingPitch = RobotContainer.PITCHER.getCurrentPitch();
        fieldRelativeNoteExitPointTranslation = calculateFieldRelativeNoteExitPoint(currentRobotPose.getTranslation(), currentRobotAngle);
        initialXYVelocity = calculateInitialXYVelocityWithRobotVelocity(startingPitch, startingTangentialVelocity, currentRobotAngle);
        initialZVelocity = startingPitch.getSin() * startingTangentialVelocity;
    }

    private double getStartingTangentialVelocity() {
        return SHOOTING_CALCULATIONS.angularVelocityToTangentialVelocity(RobotContainer.SHOOTER.getCurrentRightMotorVelocityRotationsPerSecond());
    }

    private Translation3d calculateFieldRelativeNoteExitPoint(Translation2d currentRobotPose, Rotation2d currentRobotAngle) {
        final MirrorableRotation2d robotAngle = new MirrorableRotation2d(currentRobotAngle, false);
        final Pose3d fieldRelativeNoteExitPoint = SHOOTING_CALCULATIONS.calculateShooterNoteExitPointFieldRelativePose(startingPitch, currentRobotPose, robotAngle);
        return fieldRelativeNoteExitPoint.getTranslation();
    }

    private double calculateNoteZDifference(double t) {
        return (initialZVelocity * t) + (-0.5 * ShootingConstants.G_FORCE * t * t);
    }
}
