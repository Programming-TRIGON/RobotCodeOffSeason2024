package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A 2D representation of the climber mechanism.
 */
public class ClimberVisualization {
    // sin(a)/A = sin(b)/B       * B
    // sin(a)*B/A = sin(b)
    // sin^-1(sin(a)*B/A) = b

    // a = angle of the arm // param
    // b = angle of string
    // A = length of the arm // constant
    // B = length of the string // param
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
     * Constructs a ClimberMechanism2d object.
     *
     * @param name            the name of the mechanism
     * @param firstJointColor the firstJointColor of the mechanism
     */
    public ClimberVisualization(String name, Color8Bit firstJointColor, Color8Bit stringColor, Translation3d firstJointOriginPoint) {
        this.name = name;
        this.firstJointOriginPoint = firstJointOriginPoint;
        this.key = "Mechanisms/" + name;
        this.mechanism = new Mechanism2d(2 * ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, 2 * ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS);
        // stringAngleAddition.getCos * FIRST_JOINT_TO_STRING_CONNECTION = x
        // stringAngleAddition.getSin * FIRST_JOINT_TO_STRING_CONNECTION = y
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

    }

    /**
     * Updates the current climber arm angle, the target climber arm angle, the string angle, and the target string angle, then logs the mechanism.
     *
     * @param currentClimberAngle       the current climber arm angle
     * @param targetClimberAngle        the target climber arm angle
     * @param currentStringLengthMeters the current string length in meters used to calculate the string angle
     * @param targetStringLengthMeters  the target string length in meters used to calculate the target string angle
     * @param currentState              the current climber state
     */
    public void update(Rotation2d currentClimberAngle, Rotation2d targetClimberAngle, double currentStringLengthMeters, double targetStringLengthMeters, ClimberConstants.ClimberState currentState) {
        update(currentClimberAngle, currentStringLengthMeters, currentState);
        setTargetPosition(targetClimberAngle, targetStringLengthMeters);
    }

    /**
     * Updates the current climber arm angle and the string angle, then logs the mechanism.
     *
     * @param currentClimberAngle       the current climber arm angle
     * @param currentStringLengthMeters the current string length in meters used to calculate the string angle
     * @param currentState              the current climber state
     */
    public void update(Rotation2d currentClimberAngle, double currentStringLengthMeters, ClimberConstants.ClimberState currentState) {
        currentFirstJointLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - currentClimberAngle.getDegrees());
        currentStringPositionLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - calculateStringAngle(currentClimberAngle.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION), currentStringLengthMeters).getDegrees());
        currentStringPositionLigament.setLength(currentStringLengthMeters);
        Logger.recordOutput("Poses/Components/" + name + "FirstJointPose", currentClimberAngle);
        Logger.recordOutput("Poses/Components/" + name + "secondJointPose", getClimberSecondJointPose(getClimberFirstJointPose(firstJointOriginPoint, currentClimberAngle), currentState));
        update();
    }

    /**
     * Logs the mechanism.
     */
    public void update() {
        Logger.recordOutput(key, mechanism);
    }

    /**
     * Sets the target climber arm angle and the target string angle but doesn't log the mechanism.
     *
     * @param targetClimberAngle       the target climber arm angle
     * @param targetStringLengthMeters the current string length in meters used to calculate the target string angle
     */
    public void setTargetPosition(Rotation2d targetClimberAngle, double targetStringLengthMeters) {
        targetFirstJointLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - targetClimberAngle.getDegrees());
        targetStringPositionLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - calculateStringAngle(targetClimberAngle.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION), targetStringLengthMeters).getDegrees());
        targetStringPositionLigament.setLength(targetStringLengthMeters);
    }

    private Pose3d getClimberSecondJointPose(Pose3d firstJointPose, ClimberConstants.ClimberState currentState) {
        if (currentState != ClimberConstants.ClimberState.RESTING) {
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
}
