package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.trigon.utilities.Conversions;

/**
 * A 2D representation of the climber mechanism.
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
    private Rotation2d
            climberCurrentPitch,
            climberTargetPitch;
    private ClimberConstants.ClimberState currentState;

    /**
     * Constructs a ClimberVisualization object.
     *
     * @param name                  the name of the mechanism
     * @param firstJointColor       the color of the first joint of the climber in the mechanism
     * @param stringColor           the color of the string in the mechanism
     * @param firstJointOriginPoint the first joint origin point
     */
    public ClimberVisualization(String name, Color8Bit firstJointColor, Color8Bit stringColor, Translation3d firstJointOriginPoint) {
        this.name = name;
        this.firstJointOriginPoint = firstJointOriginPoint;
        this.key = "Mechanisms/" + name;
        this.mechanism = new Mechanism2d(2 * ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, 2 * ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS);

        final MechanismRoot2d
                firstJointRoot = mechanism.getRoot(
                "FirstJointRoot",
                ClimberConstants.STRING_ANGLE_ADDITION.getCos() * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS + ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS,
                ClimberConstants.STRING_ANGLE_ADDITION.getSin() * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS),
                stringRoot = mechanism.getRoot("StringRoot", ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, 0);
        this.currentFirstJointLigament = firstJointRoot.append(new MechanismLigament2d("ZCurrentFirstJointLigament", ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, ClimberConstants.MECHANISM_STARTING_ANGLE, ClimberConstants.MECHANISM_LINE_WIDTH, firstJointColor));
        this.targetFirstJointLigament = firstJointRoot.append(new MechanismLigament2d("TargetFirstJointLigament", ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, ClimberConstants.MECHANISM_STARTING_ANGLE, ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));
        this.currentStringPositionLigament = stringRoot.append(new MechanismLigament2d("ZCurrentStringPositionLigament", ClimberConstants.CLOSED_STRING_LENGTH_METERS, ClimberConstants.CLOSED_STRING_ANGLE_DEGREES, ClimberConstants.MECHANISM_LINE_WIDTH, stringColor));
        this.targetStringPositionLigament = stringRoot.append(new MechanismLigament2d("TargetStringPositionLigament", ClimberConstants.CLOSED_STRING_LENGTH_METERS, ClimberConstants.CLOSED_STRING_ANGLE_DEGREES, ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));
        currentStringPositionLigament.append(new MechanismLigament2d("CurrentStringConnectionLigament", ClimberConstants.STRING_CONNECTION_LIGAMENT_LENGTH, ClimberConstants.STRING_CONNECTION_LIGAMENT_ANGLE, ClimberConstants.MECHANISM_LINE_WIDTH, stringColor));
        targetStringPositionLigament.append(new MechanismLigament2d("TargetStringConnectionLigament", ClimberConstants.STRING_CONNECTION_LIGAMENT_LENGTH, ClimberConstants.STRING_CONNECTION_LIGAMENT_ANGLE, ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));

        this.climberCurrentPitch = getClimberFirstJointPitch(getStringLengthMeters(0));
        this.climberTargetPitch = getClimberFirstJointPitch(getStringLengthMeters(0));
        this.currentState = ClimberConstants.ClimberState.RESTING;
    }

    /**
     * Updates the current climber arm angle and the string angle and the string length, then logs the mechanism.
     *
     * @param currentState    the current climber state used to determine the second joint pose
     * @param currentPosition the current position of the climber
     * @param targetPosition  the target position of the climber
     */
    public void update(ClimberConstants.ClimberState currentState, double currentPosition, double targetPosition) {
        this.currentState = currentState;
        currentFirstJointLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - climberCurrentPitch.getDegrees());
        currentStringPositionLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - calculateStringAngle(climberCurrentPitch.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION), getStringLengthMeters(currentPosition)).getDegrees());
        currentStringPositionLigament.setLength(getStringLengthMeters(currentPosition));
        setTargetPosition(targetPosition);
        update(currentPosition, targetPosition);
    }

    /**
     * Logs the mechanism, and updates the current pitch and target pitch.
     *
     * @param currentPosition the current position of the climber
     * @param targetPosition  the target position of the climber
     */
    public void update(double currentPosition, double targetPosition) {
        climberCurrentPitch = getClimberFirstJointPitch(getStringLengthMeters(currentPosition));
        climberTargetPitch = getClimberFirstJointPitch(getStringLengthMeters(targetPosition));
        Logger.recordOutput(key, mechanism);
        Logger.recordOutput("Poses/Components/" + name + "FirstJointPose", climberCurrentPitch);
        Logger.recordOutput("Poses/Components/" + name + "secondJointPose", getClimberSecondJointPose(getClimberFirstJointPose(firstJointOriginPoint, climberCurrentPitch), currentState));
    }

    /**
     * Sets the target climber arm angle and the target string angle but doesn't log the mechanism.
     *
     * @param targetPosition the target position of the climber
     */
    public void setTargetPosition(double targetPosition) {
        targetFirstJointLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - climberTargetPitch.getDegrees());
        targetStringPositionLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - calculateStringAngle(climberTargetPitch.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION), getStringLengthMeters(targetPosition)).getDegrees());
        targetStringPositionLigament.setLength(getStringLengthMeters(targetPosition));
    }

    private Pose3d getClimberSecondJointPose(Pose3d firstJointPose, ClimberConstants.ClimberState currentState) {
        if (currentState.affectedByRobotWeight) {
            Transform3d climberTransform = new Transform3d(
                    new Translation3d(0, ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, 0),
                    new Rotation3d(0, 90, 0)
            );
            return firstJointPose.transformBy(climberTransform);
        }
        Transform3d climberTransform = new Transform3d(
                new Translation3d(0, ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, 0),
                new Rotation3d(0, 0, 0)
        );
        return firstJointPose.transformBy(climberTransform);
    }

    private Pose3d getClimberFirstJointPose(Translation3d originPoint, Rotation2d firstJointPitch) {
        return new Pose3d(
                originPoint,
                new Rotation3d(0, firstJointPitch.getRadians(), 0)
        );
    }

    private Rotation2d calculateStringAngle(Rotation2d climberAngle, double stringLengthMeters) {
        return Rotation2d.fromRadians(
                Math.asin(climberAngle.getSin() * stringLengthMeters / ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS) + ClimberConstants.STRING_ANGLE_ADDITION.getRadians()
        );
    }

    Rotation2d getClimberCurrentPitch() {
        return climberCurrentPitch;
    }

    private Rotation2d getClimberFirstJointPitch(double stringLength) {
        final double numeratorCalculation =
                Math.pow(ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS, 2)
                        + Math.pow(ClimberConstants.FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS, 2)
                        - Math.pow(stringLength, 2);
        final double denominatorCalculation = 2 * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS * ClimberConstants.FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS;
        final double division = numeratorCalculation / denominatorCalculation;
        final double angle = Math.acos(division) + ClimberConstants.FIRST_JOINT_ANGLE_ADDITION.getRadians();
        return Rotation2d.fromRadians(angle);
    }

    private double getStringLengthMeters(double position) {
        return toMeters(position) + ClimberConstants.STRING_LENGTH_ADDITION_METERS;
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberConstants.DRUM_DIAMETER_METERS);
    }
}
