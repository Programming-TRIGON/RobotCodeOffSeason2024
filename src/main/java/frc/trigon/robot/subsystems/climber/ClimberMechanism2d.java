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
            currentClimberPositionLigament,
            targetClimberPositionLigament,
            currentStringPositionLigament,
            targetStringPositionLigament;

    /**
     * Constructs a ClimberMechanism2d object.
     *
     * @param name  the name of the mechanism
     * @param color the color of the mechanism
     */
    public ClimberMechanism2d(String name, Color8Bit color) {
        this.key = "Mechanisms/" + name;
        this.mechanism = new Mechanism2d(ClimberConstants.DISTANCE_BETWEEN_JOINTS, ClimberConstants.DISTANCE_BETWEEN_JOINTS);

        final MechanismRoot2d
                climberRoot = mechanism.getRoot("ClimberRoot", ClimberConstants.DISTANCE_BETWEEN_JOINTS, ClimberConstants.DRUM_TO_CLIMBER_FIRST_JOINT_Z_DISTANCE_METERS),
                stringRoot = mechanism.getRoot("StringRoot", ClimberConstants.DISTANCE_BETWEEN_JOINTS / 2, 0);
        this.currentClimberPositionLigament = climberRoot.append(new MechanismLigament2d("ZCurrentClimberPositionLigament", ClimberConstants.DISTANCE_BETWEEN_JOINTS, 0, ClimberConstants.MECHANISM_LINE_WIDTH, color));
        this.targetClimberPositionLigament = climberRoot.append(new MechanismLigament2d("TargetClimberPositionLigament", ClimberConstants.DISTANCE_BETWEEN_JOINTS, 0, ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));
        this.currentStringPositionLigament = stringRoot.append(new MechanismLigament2d("ZCurrentStringPositionLigament", ClimberConstants.CLOSED_STRING_LENGTH, ClimberConstants.CLOSED_STRING_ANGLE_DEGREES, ClimberConstants.MECHANISM_LINE_WIDTH, color));
        this.targetStringPositionLigament = stringRoot.append(new MechanismLigament2d("TargetStringPositionLigament", ClimberConstants.CLOSED_STRING_LENGTH, ClimberConstants.CLOSED_STRING_ANGLE_DEGREES, ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));
    }

    /**
     * Updates the current climber arm angle, the target climber arm angle, the string angle, and the target string angle, then logs the mechanism.
     *
     * @param currentClimberAngle the current climber arm angle
     * @param targetClimberAngle  the target climber arm angle
     * @param currentStringLength the current string length used to calculate the string angle
     * @param targetStringLength  the target string length used to calculate the target string angle
     */
    public void update(Rotation2d currentClimberAngle, Rotation2d targetClimberAngle, double currentStringLength, double targetStringLength) {
        update(currentClimberAngle, currentStringLength);
        setTargetPosition(targetClimberAngle, targetStringLength);
    }

    /**
     * Updates the current climber arm angle and the string angle, then logs the mechanism.
     *
     * @param currentClimberAngle the current climber arm angle
     * @param currentStringLength the current string length used to calculate the string angle
     */
    public void update(Rotation2d currentClimberAngle, double currentStringLength) {
        currentClimberPositionLigament.setAngle(currentClimberAngle.getDegrees());
        currentStringPositionLigament.setAngle(calculateStringAngle(currentClimberAngle, currentStringLength).getDegrees());
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
     * @param targetClimberAngle the target climber arm angle
     * @param targetStringLength the current string length used to calculate the target string angle
     */
    public void setTargetPosition(Rotation2d targetClimberAngle, double targetStringLength) {
        targetClimberPositionLigament.setAngle(targetClimberAngle.getDegrees());
        targetStringPositionLigament.setAngle(calculateStringAngle(targetClimberAngle, targetStringLength + ClimberConstants.STRING_LENGTH_ADDITION).getDegrees());
    }


    private Rotation2d calculateStringAngle(Rotation2d climberAngle, double stringLength) {
        return Rotation2d.fromRadians(
                Math.asin(climberAngle.getSin() * stringLength / ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS)
        );
    }
}
