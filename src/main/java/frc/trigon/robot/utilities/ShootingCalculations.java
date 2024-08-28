package frc.trigon.robot.utilities;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.constants.ShootingConstants;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
import frc.trigon.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.mirrorable.MirrorableRotation2d;
import org.trigon.utilities.mirrorable.MirrorableTranslation3d;

public class ShootingCalculations {
    private static ShootingCalculations INSTANCE = null;
    private TargetShootingState targetShootingState = new TargetShootingState(new MirrorableRotation2d(new Rotation2d(), false), new Rotation2d(), 0);

    public static ShootingCalculations getInstance() {
        if (INSTANCE == null)
            INSTANCE = new ShootingCalculations();
        return INSTANCE;
    }

    private ShootingCalculations() {
    }

    /**
     * Updates the {@linkplain ShootingCalculations#targetShootingState} class variable to contain the target state for delivery.
     */
    public void updateCalculationsForDelivery() {
        Logger.recordOutput("ShootingCalculations/TargetDeliveryPose", FieldConstants.TARGET_DELIVERY_POSITION.get());
        targetShootingState = calculateTargetShootingState(FieldConstants.TARGET_DELIVERY_POSITION, ShootingConstants.DELIVERY_STANDING_VELOCITY_ROTATIONS_PER_SECOND, true);
    }

    /**
     * Updates the {@linkplain ShootingCalculations#targetShootingState} class variable to contain the target state for shooting at the speaker.
     */
    public void updateCalculationsForSpeakerShot() {
        Logger.recordOutput("ShootingCalculations/TargetSpeakerPose", FieldConstants.SPEAKER_TRANSLATION.get());
        targetShootingState = calculateTargetShootingState(FieldConstants.SPEAKER_TRANSLATION, ShootingConstants.SPEAKER_SHOT_STANDING_VELOCITY_ROTATIONS_PER_SECOND, false);
    }

    /**
     * @return the target state of the robot to shoot at the provided shooting target
     */
    public TargetShootingState getTargetShootingState() {
        return targetShootingState;
    }

    /**
     * Converts a given shooter's angular velocity to the shooter's tangential velocity.
     * This is specific for our shooter's specs.
     *
     * @param angularVelocity the angular velocity of the shooter
     * @return the tangential velocity of the shooter
     */
    public double angularVelocityToTangentialVelocity(double angularVelocity) {
        return Conversions.rotationsToDistance(angularVelocity, ShooterConstants.WHEEL_DIAMETER_METERS);
    }

    /**
     * Converts a given shooter's tangential velocity to the shooter's angular velocity.
     * This is specific for our shooter's specs.
     *
     * @param tangentialVelocity the tangential velocity of the shooter
     * @return the angular velocity of the shooter
     */
    public double tangentialVelocityToAngularVelocity(double tangentialVelocity) {
        return Conversions.distanceToRotations(tangentialVelocity, ShooterConstants.WHEEL_DIAMETER_METERS);
    }

    /**
     * @return the robot's velocity relative to field in the xy plane
     */
    public Translation3d getRobotFieldRelativeVelocity() {
        final ChassisSpeeds fieldRelativeSpeeds = RobotContainer.SWERVE.getFieldRelativeVelocity();
        return new Translation3d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond, 0);
    }

    /**
     * Predicts where the robot will be in a given amount of time on the xy plane.
     *
     * @param predictionTime the amount of time to predict the robot's position in, in seconds
     * @return the predicted position of the robot
     */
    private Translation2d predictFutureTranslation(double predictionTime) {
        final Translation2d fieldRelativeVelocity = getRobotFieldRelativeVelocity().toTranslation2d();
        final Translation2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getTranslation();
        return currentPose.plus(fieldRelativeVelocity.times(predictionTime));
    }

    /**
     * Calculates the necessary pitch, robot yaw, and shooting velocity in order to shoot at the shooting target.
     *
     * @param shootingTarget                             the point we want the note reach
     * @param standingShootingVelocityRotationsPerSecond the shooting velocity to calculate optimal pitch from, when the robot isn't moving.
     *                                                   This may change if the robot's velocity is not 0, but will act as a starting point
     * @param reachFromAbove                             should we reach to point from above, with an arch, or from below, as fast as possible
     *                                                   Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @return the target state of the robot so the note will reach the shooting target, as a {@linkplain ShootingCalculations.TargetShootingState}
     */
    private TargetShootingState calculateTargetShootingState(MirrorableTranslation3d shootingTarget, double standingShootingVelocityRotationsPerSecond, boolean reachFromAbove) {
        final Translation2d currentTranslation = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getTranslation();
        final MirrorableRotation2d standingTargetRobotAngle = getAngleToTarget(currentTranslation, shootingTarget);
        final double standingTangentialVelocity = angularVelocityToTangentialVelocity(standingShootingVelocityRotationsPerSecond);
        final Rotation2d standingTargetPitch = calculateTargetPitch(standingTangentialVelocity, reachFromAbove, currentTranslation, standingTargetRobotAngle, shootingTarget);
        final TargetShootingState standingShootingState = new TargetShootingState(standingTargetRobotAngle, standingTargetPitch, standingTangentialVelocity);

        return calculateTargetShootingState(standingShootingState);
    }

    /**
     * Calculates the necessary pitch, robot yaw, and shooting velocity in order to shoot at the shooting target.
     * This will calculate the TargetShootingState from a standing shooting state, to a dynamic "moving" shooting state to account for robot velocity.
     * The calculation is done using vector subtraction, where we subtract the robot's 3d vector from the note's 3d vector, and then find the new state from the end vector.
     * Note this actually uses vector addition since the shooter is placed in the opposite direction of the robot's velocity.
     *
     * @param standingShootingState the standing shooting state to calculate off of
     * @return the target state of the robot so the note will reach the shooting target, as a {@linkplain ShootingCalculations.TargetShootingState}
     */
    private TargetShootingState calculateTargetShootingState(TargetShootingState standingShootingState) {
        final Translation3d noteVector = new Translation3d(standingShootingState.targetShootingVelocityRotationsPerSecond, new Rotation3d(0, -standingShootingState.targetPitch.getRadians(), standingShootingState.targetRobotAngle.get().getRadians()));
        final Translation3d robotVector = getRobotFieldRelativeVelocity();
        final Translation3d shootingVector = noteVector.plus(robotVector);

        return calculateTargetShootingState(shootingVector);
    }

    /**
     * Calculates the necessary pitch, robot yaw, and shooting velocity in order to shoot at the shooting target.
     * This will create the TargetShootingState from a shooting vector that was predetermined (most likely by a prior vector subtraction of the note and robot velocity vectors).
     *
     * @param shootingVector the predetermined shooting vector
     * @return the target state of the robot so the note will reach the shooting target, as a {@linkplain ShootingCalculations.TargetShootingState}
     */
    private TargetShootingState calculateTargetShootingState(Translation3d shootingVector) {
        final MirrorableRotation2d targetRobotAngle = new MirrorableRotation2d(getYaw(shootingVector), false);
        final Rotation2d targetPitch = getPitch(shootingVector);
        final double targetVelocity = tangentialVelocityToAngularVelocity(shootingVector.getNorm());

        return new TargetShootingState(targetRobotAngle, targetPitch, targetVelocity);
    }

    /**
     * Extracts the yaw off of a 3d vector.
     *
     * @param vector the vector to extract the yaw from
     * @return the yaw of the vector
     */
    private Rotation2d getYaw(Translation3d vector) {
        return new Rotation2d(vector.getX(), vector.getY());
    }

    /**
     * Extracts the pitch off of a 3d vector.
     *
     * @param vector the vector to extract the pitch from
     * @return the pitch of the vector
     */
    private Rotation2d getPitch(Translation3d vector) {
        return new Rotation2d(Math.atan2(vector.getZ(), Math.hypot(vector.getX(), vector.getY())));
    }

    /**
     * Calculates the optimal pitch for the given parameters, using the Projectile Motion calculation.
     *
     * @param noteTangentialVelocity the exit velocity of the note, as tangential velocity
     * @param reachFromAbove         should we reach to point from above, with an arch, or from below, as fast as possible
     *                               Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @param currentTranslation     the current translation of the robot, to find the note exit point's pose with
     * @param targetRobotAngle       the previously calculated target robot angle, to find the note exit point's pose with
     * @param shootingTarget         the point we want the note reach
     * @return the pitch the pitcher should reach in order to shoot to the shooting target
     */
    private Rotation2d calculateTargetPitch(double noteTangentialVelocity, boolean reachFromAbove, Translation2d currentTranslation, MirrorableRotation2d targetRobotAngle, MirrorableTranslation3d shootingTarget) {
        final Pose3d noteExitPointFieldRelativePose = calculateShooterNoteExitPointFieldRelativePose(RobotContainer.PITCHER.getTargetPitch(), currentTranslation, targetRobotAngle);
        final double noteExitPointXYDistanceFromShootingTarget = noteExitPointFieldRelativePose.getTranslation().toTranslation2d().getDistance(shootingTarget.get().toTranslation2d());
        final double noteExitPointHeightDifferenceFromTarget = shootingTarget.get().getZ() - noteExitPointFieldRelativePose.getZ();
        return calculateTargetPitchUsingProjectileMotion(noteTangentialVelocity, noteExitPointXYDistanceFromShootingTarget, noteExitPointHeightDifferenceFromTarget, reachFromAbove);
    }

    /**
     * Calculates the pitch the pitcher should reach in order to shoot at the shooting target using projectile motion.
     * This will fully calculate the target pitch using physics.
     *
     * @param noteTangentialVelocity                           the tangential velocity of the shooter
     * @param shooterNoteExitPointXYDistanceFromShootingTarget the xy distance from the shooting target to the shooter's note exit point on the xy plane
     * @param noteExitPointHeightDifferenceFromTarget          the height difference between the shooter's note exit point and the shooting target
     * @param reachFromAbove                                   should we reach to point from above, with an arch, or from below, as fast as possible
     *                                                         Shooting from above is useful for actions like delivery, whereas shooting from below is useful when we don't want to come from above, and in our case touch the upper speaker
     * @return the pitch the robot should reach in order to shoot at the shooting target
     * @link <a href="https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)">Projectile Motion</a>
     */
    private Rotation2d calculateTargetPitchUsingProjectileMotion(double noteTangentialVelocity, double shooterNoteExitPointXYDistanceFromShootingTarget, double noteExitPointHeightDifferenceFromTarget, boolean reachFromAbove) {
        Logger.recordOutput("ShootingCalculations/NoteTangentialVelocity", noteTangentialVelocity);
        Logger.recordOutput("ShootingCalculations/ShooterNoteExitPointHeight", noteExitPointHeightDifferenceFromTarget);
        final double gForce = ShootingConstants.G_FORCE;
        final double velocitySquared = Math.pow(noteTangentialVelocity, 2);
        final double velocity4thPower = Math.pow(noteTangentialVelocity, 4);
        final double distanceSquared = Math.pow(shooterNoteExitPointXYDistanceFromShootingTarget, 2);
        final double squareRoot = Math.sqrt(
                velocity4thPower - (gForce * ((gForce * distanceSquared) + (2 * velocitySquared * noteExitPointHeightDifferenceFromTarget)))
        );
        final double numerator = reachFromAbove ? velocitySquared + squareRoot : velocitySquared - squareRoot;
        final double denominator = gForce * shooterNoteExitPointXYDistanceFromShootingTarget;
        final double fraction = numerator / denominator;
        double angleRadians = Math.atan(fraction);
        if (Double.isNaN(angleRadians) || Double.isInfinite(angleRadians) || angleRadians < 0)
            angleRadians = PitcherConstants.DEFAULT_PITCH.getRadians();
        Logger.recordOutput("ShootingCalculations/TargetPitch", Math.toDegrees(angleRadians));
        return Rotation2d.fromRadians(angleRadians);
    }

    /**
     * Calculates the shooter's note exit point's 3d pose on the field from the given parameters.
     * The note exit point is the furthest point of the shooter from the pivot point,
     * and where the note leaves the shooter.
     *
     * @param pitcherAngle       the pitcher angle, to base off from
     * @param currentTranslation the field relative current translation, to base off from
     * @param robotAngle         the robot angle, to base off from
     * @return the shooter's note exit point's 3d pose on the field
     */
    public Pose3d calculateShooterNoteExitPointFieldRelativePose(Rotation2d pitcherAngle, Translation2d currentTranslation, MirrorableRotation2d robotAngle) {
        final Pose3d noteExitPointSelfRelativePose = calculateShooterNoteExitPointSelfRelativePose(pitcherAngle);
        final Transform3d robotToNoteExitPoint = noteExitPointSelfRelativePose.minus(new Pose3d());
        final Pose3d currentPose = new Pose3d(new Pose2d(currentTranslation, robotAngle.get()));
        return currentPose.transformBy(robotToNoteExitPoint);
    }

    /**
     * Calculates the shooter's note exit point's 3d pose relative to the robot.
     * The note exit point is the furthest point of the shooter from the pivot point,
     * and where the note leaves the shooter.
     *
     * @param pitcherAngle the pitcher angle, to base off from
     * @return the shooter's note exit point's 3d pose relative to the robot
     */
    private Pose3d calculateShooterNoteExitPointSelfRelativePose(Rotation2d pitcherAngle) {
        final Pose3d pivotPoint = ShootingConstants.ROBOT_RELATIVE_PITCHER_PIVOT_POINT.transformBy(new Transform3d(new Translation3d(), new Rotation3d(0, -pitcherAngle.getRadians(), 0)));
        return pivotPoint.plus(ShootingConstants.PITCHER_PIVOT_POINT_TO_NOTE_EXIT_POSITION);
    }

    /**
     * Uses {@linkplain java.lang.Math#atan2} to calculate the angle to face the shooting target.
     *
     * @param currentTranslation the current translation of the robot
     * @param shootingTarget     the shootingTarget of the shooting. What we want the projectile to reach
     * @return the angle (yaw) the robot should reach in order to face the shooting target
     */
    private MirrorableRotation2d getAngleToTarget(Translation2d currentTranslation, MirrorableTranslation3d shootingTarget) {
        final Translation2d difference = currentTranslation.minus(shootingTarget.get().toTranslation2d());
        return MirrorableRotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()), false);
    }

    public record TargetShootingState(MirrorableRotation2d targetRobotAngle, Rotation2d targetPitch,
                                      double targetShootingVelocityRotationsPerSecond) {
    }
}