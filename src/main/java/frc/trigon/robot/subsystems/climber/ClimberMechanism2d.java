package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

/**
 * A 2D representation of the climber mechanism.
 */
public class ClimberMechanism2d {
    // sin(a)/A = sin(b)/B       * B
    // sin(a)*B/A = sin(b)
    // sin^-1(sin(a)*B/A) = b

    // a = angle of the arm // param
    // b = angle of string
    // A = length of the arm // constant
    // B = length of the string // param
    private final String key;
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
    public ClimberMechanism2d(String name, Color8Bit firstJointColor, Color8Bit stringColor) {
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
     */
    public void update(Rotation2d currentClimberAngle, Rotation2d targetClimberAngle, double currentStringLengthMeters, double targetStringLengthMeters) {
        update(currentClimberAngle, currentStringLengthMeters);
        setTargetPosition(targetClimberAngle, targetStringLengthMeters);
    }

    /**
     * Updates the current climber arm angle and the string angle, then logs the mechanism.
     *
     * @param currentClimberAngle       the current climber arm angle
     * @param currentStringLengthMeters the current string length in meters used to calculate the string angle
     */
    public void update(Rotation2d currentClimberAngle, double currentStringLengthMeters) {
        currentFirstJointLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - currentClimberAngle.getDegrees());
        currentStringPositionLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - calculateStringAngle(currentClimberAngle.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION), currentStringLengthMeters).getDegrees());
        currentStringPositionLigament.setLength(currentStringLengthMeters);
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


    private Rotation2d calculateStringAngle(Rotation2d climberAngle, double stringLengthMeters) {
        return Rotation2d.fromRadians(
                Math.asin(climberAngle.getSin() * stringLengthMeters / ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS) + ClimberConstants.STRING_ANGLE_ADDITION.getRadians()
        );
    }
}
