package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.trigon.utilities.Conversions;

/**
 * A class that visualizes the climber, using a Mechanism2d, and calculating the 3d poses of the climber's joints for Advantage Scope.
 */
public class ClimberVisualization {
    private final String
            name,
            key;
    private final Translation3d firstJointOriginPoint;
    private final Mechanism2d mechanism;
    private final MechanismLigament2d
            currentFirstJointLigament,
            targetFirstJointLigament,
            currentStringPositionLigament,
            targetStringPositionLigament;

    /**
     * Constructs a ClimberVisualization object.
     *
     * @param name                  the name of climber (left of right)
     * @param firstJointColor       the color of the first joint of the climber in the 2d mechanism
     * @param stringColor           the color of the string in the 2d mechanism
     * @param firstJointOriginPoint the first joint origin point
     */
    public ClimberVisualization(String name, Color8Bit firstJointColor, Color8Bit stringColor, Translation3d firstJointOriginPoint) {
        this.name = name;
        this.firstJointOriginPoint = firstJointOriginPoint;
        this.key = "Mechanisms/" + name;
        this.mechanism = new Mechanism2d(2 * ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, 2 * ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS);

        final MechanismRoot2d firstJointRoot = mechanism.getRoot(
                "FirstJointRoot",
                ClimberConstants.STRING_ANGLE_ADDITION.getCos() * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS + ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS,
                ClimberConstants.STRING_ANGLE_ADDITION.getSin() * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS
        );
        final MechanismRoot2d stringRoot = mechanism.getRoot("StringRoot", ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, 0);

        this.currentFirstJointLigament = firstJointRoot.append(new MechanismLigament2d("ZCurrentFirstJointLigament", ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, ClimberConstants.MECHANISM_STARTING_ANGLE.getDegrees(), ClimberConstants.MECHANISM_LINE_WIDTH, firstJointColor));
        this.targetFirstJointLigament = firstJointRoot.append(new MechanismLigament2d("TargetFirstJointLigament", ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, ClimberConstants.MECHANISM_STARTING_ANGLE.getDegrees(), ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));
        this.currentStringPositionLigament = stringRoot.append(new MechanismLigament2d("ZCurrentStringPositionLigament", ClimberConstants.STRING_LENGTH_ADDITION_METERS, ClimberConstants.CLOSED_STRING_ANGLE.getDegrees(), ClimberConstants.MECHANISM_LINE_WIDTH, stringColor));
        this.targetStringPositionLigament = stringRoot.append(new MechanismLigament2d("TargetStringPositionLigament", ClimberConstants.STRING_LENGTH_ADDITION_METERS, ClimberConstants.CLOSED_STRING_ANGLE.getDegrees(), ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));
    }

    /**
     * Updates the 2d mechanism's current climber arm angle, string angle, and string length. Then calculates the 3d Advantage Scope poses, and logs 2d mechanism and 3d poses.
     *
     * @param targetState              the target climber state used to determine the second joint's 3d pose
     * @param currentPositionRotations the current position of the climber
     * @param targetPositionRotations  the target position of the climber
     */
    public void update(ClimberConstants.ClimberState targetState, double currentPositionRotations, double targetPositionRotations) {
        final double currentStringLength = calculateStringLengthMeters(currentPositionRotations);
        final double targetStringLength = calculateStringLengthMeters(targetPositionRotations);
        final Rotation2d currentFirstJointPitch = calculateFirstJointPitch(currentStringLength);
        final Rotation2d targetFirstJointPitch = calculateFirstJointPitch(targetStringLength);
        final Rotation2d currentStringAngle = calculateStringAngle(currentFirstJointPitch, currentStringLength);
        final Rotation2d targetStringAngle = calculateStringAngle(targetFirstJointPitch, targetStringLength);

        setCurrentMechanism2dState(currentFirstJointPitch, currentStringAngle, currentStringLength);
        setTargetMechanism2dState(targetFirstJointPitch, targetStringAngle, targetStringLength);
        log(currentFirstJointPitch, targetState);
    }

    /**
     * Sets the current state of the 2d mechanism.
     *
     * @param currentFistJointPitch the current pitch of the first joint
     * @param currentStringAngle    the current string angle
     * @param currentStringLength   the current string length
     */
    private void setCurrentMechanism2dState(Rotation2d currentFistJointPitch, Rotation2d currentStringAngle, double currentStringLength) {
        currentFirstJointLigament.setAngle(flipAngle(currentFistJointPitch));
        currentStringPositionLigament.setAngle(currentStringAngle);
        currentStringPositionLigament.setLength(currentStringLength);

    }

    /**
     * Sets the target state of the 2d mechanism.
     *
     * @param targetFirstJointPitch the target pitch of the first joint
     * @param targetStringAngle     the target string angle
     * @param targetStringLength    the target string length
     */
    private void setTargetMechanism2dState(Rotation2d targetFirstJointPitch, Rotation2d targetStringAngle, double targetStringLength) {
        targetFirstJointLigament.setAngle(flipAngle(targetFirstJointPitch));
        targetStringPositionLigament.setAngle(targetStringAngle);
        targetStringPositionLigament.setLength(targetStringLength);
    }

    /**
     * Logs the 2d mechanism and the 3d Advantage scope poses.
     *
     * @param currentFirstJointPitch the current pitch of the first joint
     * @param targetState            the target state
     */
    private void log(Rotation2d currentFirstJointPitch, ClimberConstants.ClimberState targetState) {
        final Pose3d firstJointPose = calculateFirstJointPose(firstJointOriginPoint, currentFirstJointPitch);
        Logger.recordOutput(key, mechanism);
        Logger.recordOutput("Poses/Components/" + name + "FirstJointPose", firstJointPose);
        Logger.recordOutput("Poses/Components/" + name + "SecondJointPose", calculateSecondJointPose(firstJointPose, targetState, currentFirstJointPitch));
    }


    private Pose3d calculateSecondJointPose(Pose3d firstJointPose, ClimberConstants.ClimberState currentState, Rotation2d firstJointPitch) {
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(-ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, 0, 0),
                new Rotation3d(0, currentState.affectedByRobotWeight ? ClimberConstants.SECOND_JOINT_ON_CHAIN_PITCH.minus(firstJointPitch).getRadians() : 0, 0)
        );
        return firstJointPose.transformBy(climberTransform);
    }

    private Pose3d calculateFirstJointPose(Translation3d originPoint, Rotation2d firstJointPitch) {
        return new Pose3d(
                originPoint,
                new Rotation3d(0, firstJointPitch.getRadians(), 0)
        );
    }

    private Rotation2d calculateStringAngle(Rotation2d firstJointAngle, double stringLengthMeters) {
        final double firstJointSin = firstJointAngle.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION).getSin();
        final double division = (firstJointSin * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS) / stringLengthMeters;
        final double asin = Math.asin(division);
        return Rotation2d.fromRadians(asin + ClimberConstants.STRING_ANGLE_ADDITION.getRadians());
    }

    private Rotation2d calculateFirstJointPitch(double stringLengthMeters) {
        final double numeratorCalculation =
                ((ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS)
                        + (ClimberConstants.FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS * ClimberConstants.FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS))
                        - (stringLengthMeters * stringLengthMeters);
        final double denominatorCalculation = 2 * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS * ClimberConstants.FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS;
        final double division = numeratorCalculation / denominatorCalculation;
        final double angle = Math.acos(division) + ClimberConstants.FIRST_JOINT_ANGLE_ADDITION.getRadians();
        return Rotation2d.fromRadians(angle);
    }

    private double calculateStringLengthMeters(double positionRotations) {
        return toMeters(positionRotations) + ClimberConstants.STRING_LENGTH_ADDITION_METERS;
    }

    /**
     * Flips the angle.
     *
     * @param angle the angle to flip
     * @return the flipped angle
     */
    private Rotation2d flipAngle(Rotation2d angle) {
        return ClimberConstants.MECHANISM_STARTING_ANGLE.minus(angle);
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberConstants.DRUM_DIAMETER_METERS);
    }
}
