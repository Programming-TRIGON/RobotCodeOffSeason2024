package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
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
    private Rotation2d startingPitch, startingYaw;
    private Translation2d initialXYVelocity;
    private double initialZVelocity;
    private Pose3d notePose = new Pose3d();
    private boolean hasNoteCrossedAmpY = false;

    @Override
    public void initialize() {
        configureStartingStats();
    }

    @Override
    public void execute() {
        final double timeDifference = Timer.getFPGATimestamp() - startingTimeSeconds;

        final Transform3d noteTransform = calculateNoteTransform(timeDifference);
        notePose = new Pose3d(fieldRelativeNoteExitPointTranslation, new Rotation3d()).plus(noteTransform);

        if (didNoteJustCrossAmpY(notePose.getTranslation().getY()))
            configureNoteInAmpStats();

        Logger.recordOutput("Poses/GamePieces/ShotNotePose", notePose);
    }

    @Override
    public boolean isFinished() {
        return notePose.getZ() < 0;
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Poses/GamePieces/ShotNotePose", new Pose3d(0, 0, 50, new Rotation3d()));
    }

    private Transform3d calculateNoteTransform(double timeDifference) {
        return new Transform3d(calculateNoteXDifference(timeDifference), calculateNoteYDifference(timeDifference), calculateNoteZDifference(timeDifference), new Rotation3d(0, -startingPitch.getRadians(), startingYaw.getRadians()));
    }

    private double calculateNoteXDifference(double t) {
        return initialXYVelocity.getX() * t;
    }

    private double calculateNoteYDifference(double t) {
        return initialXYVelocity.getY() * t;
    }

    private double calculateNoteZDifference(double t) {
        return (initialZVelocity * t) + (-0.5 * ShootingConstants.G_FORCE * t * t);
    }

    private void configureStartingStats() {
        startingTimeSeconds = Timer.getFPGATimestamp();

        final Pose2d currentRobotPose = RobotContainer.POSE_ESTIMATOR.getCurrentEstimatedPose();
        final Rotation2d currentRobotAngle = currentRobotPose.getRotation();
        final double startingTangentialVelocity = getStartingTangentialVelocity();

        startingPitch = RobotContainer.PITCHER.getCurrentPitch();
        startingYaw = currentRobotAngle.minus(new Rotation2d(Math.PI));
        fieldRelativeNoteExitPointTranslation = calculateFieldRelativeNoteExitPoint(currentRobotPose.getTranslation(), currentRobotAngle);
        initialXYVelocity = calculateInitialXYVelocityWithRobotVelocity(startingPitch, startingTangentialVelocity, currentRobotAngle);
        initialZVelocity = startingPitch.getSin() * startingTangentialVelocity;
    }

    private boolean didNoteJustCrossAmpY(double noteY) {
        return noteY > FieldConstants.FIELD_WIDTH_METERS && !hasNoteCrossedAmpY;
    }

    private void configureNoteInAmpStats() {
        hasNoteCrossedAmpY = true;
        startingTimeSeconds = Timer.getFPGATimestamp();

        startingPitch = Rotation2d.fromDegrees(90);
        startingYaw = Rotation2d.fromDegrees(90);
        fieldRelativeNoteExitPointTranslation = new Translation3d(
                notePose.getX(),
                0,
                notePose.getZ());
        initialXYVelocity = new Translation2d(0, 0);
        initialZVelocity = 0;
    }

    private double getStartingTangentialVelocity() {
        return SHOOTING_CALCULATIONS.angularVelocityToTangentialVelocity(RobotContainer.SHOOTER.getCurrentRightMotorVelocityRotationsPerSecond());
    }

    private Translation3d calculateFieldRelativeNoteExitPoint(Translation2d currentRobotPose, Rotation2d currentRobotAngle) {
        final MirrorableRotation2d robotAngle = new MirrorableRotation2d(currentRobotAngle, false);
        final Pose3d fieldRelativeNoteExitPoint = SHOOTING_CALCULATIONS.calculateShooterNoteExitPointFieldRelativePose(startingPitch, currentRobotPose, robotAngle);
        return fieldRelativeNoteExitPoint.getTranslation();
    }

    private Translation2d calculateInitialXYVelocityWithRobotVelocity(Rotation2d startingPitch, double startingTangentialVelocity, Rotation2d currentRobotAngle) {
        final Translation2d robotFieldRelativeVelocity = SHOOTING_CALCULATIONS.getRobotFieldRelativeVelocity().toTranslation2d();
        final double noteXYVelocityRelativeToRobot = startingPitch.getCos() * startingTangentialVelocity;
        return robotFieldRelativeVelocity.plus(new Translation2d(noteXYVelocityRelativeToRobot, currentRobotAngle.minus(new Rotation2d(Math.PI))));
    }
}
