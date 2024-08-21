package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.trigon.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

/**
 * A class that visualizes the climber in the simulation, using a mechanism and calculating the poses.
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
        this.currentFirstJointLigament = firstJointRoot.append(new MechanismLigament2d("ZCurrentFirstJointLigament", ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, ClimberConstants.MECHANISM_STARTING_ANGLE.getDegrees(), ClimberConstants.MECHANISM_LINE_WIDTH, firstJointColor));
        this.targetFirstJointLigament = firstJointRoot.append(new MechanismLigament2d("TargetFirstJointLigament", ClimberConstants.DISTANCE_BETWEEN_JOINTS_METERS, ClimberConstants.MECHANISM_STARTING_ANGLE.getDegrees(), ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));
        this.currentStringPositionLigament = stringRoot.append(new MechanismLigament2d("ZCurrentStringPositionLigament", ClimberConstants.CLOSED_STRING_LENGTH_METERS, ClimberConstants.CLOSED_STRING_ANGLE.getDegrees(), ClimberConstants.MECHANISM_LINE_WIDTH, stringColor));
        this.targetStringPositionLigament = stringRoot.append(new MechanismLigament2d("TargetStringPositionLigament", ClimberConstants.CLOSED_STRING_LENGTH_METERS, ClimberConstants.CLOSED_STRING_ANGLE.getDegrees(), ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));
        currentStringPositionLigament.append(new MechanismLigament2d("CurrentStringConnectionLigament", ClimberConstants.STRING_CONNECTION_LIGAMENT_LENGTH, ClimberConstants.STRING_CONNECTION_LIGAMENT_ANGLE_DEGREES, ClimberConstants.MECHANISM_LINE_WIDTH, stringColor));
        targetStringPositionLigament.append(new MechanismLigament2d("TargetStringConnectionLigament", ClimberConstants.STRING_CONNECTION_LIGAMENT_LENGTH, ClimberConstants.STRING_CONNECTION_LIGAMENT_ANGLE_DEGREES, ClimberConstants.MECHANISM_LINE_WIDTH, ClimberConstants.GRAY));

        this.currentState = ClimberConstants.ClimberState.RESTING;
    }

    /**
     * Updates the current climber arm angle, string angle, and string length, then logs the mechanism.
     *
     * @param targetState              the current climber state used to determine the second joint pose
     * @param currentPositionRotations the current position of the climber
     * @param targetPositionRotations  the target position of the climber
     */
    public void update(ClimberConstants.ClimberState targetState, double currentPositionRotations, double targetPositionRotations) {
        this.currentState = targetState;

        final Rotation2d currentFirstJointPitch = calculateClimberFirstJointPitch(calculateStringLengthMeters(currentPositionRotations));
        final Rotation2d targetFirstJointPitch = calculateClimberFirstJointPitch(calculateStringLengthMeters(targetPositionRotations));
        final double currentStringLength = calculateStringLengthMeters(currentPositionRotations);
        final double targetStringLength = calculateStringLengthMeters(targetPositionRotations);
        final Rotation2d currentStringAngle = calculateStringAngle(currentFirstJointPitch.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION), currentStringLength);
        final Rotation2d targetStringAngle = calculateStringAngle(targetFirstJointPitch.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION), targetStringLength);

        setCurrentMechanism2dState(currentFirstJointPitch, currentStringAngle, currentStringLength);
        setTargetMechanism2dState(targetFirstJointPitch, targetStringAngle, targetStringLength);
        log(currentFirstJointPitch);
    }

    private void setCurrentMechanism2dState(Rotation2d currentFistJointPitch, Rotation2d currentStringAngle, double currentStringLength) {
        currentFirstJointLigament.setAngle(flipAngle(currentFistJointPitch));
        currentStringPositionLigament.setAngle(flipAngle(currentStringAngle));
        currentStringPositionLigament.setLength(currentStringLength);

    }

    /**
     * Sets the target state of the mechanism, but doesn't log the mechanism.
     *
     * @param targetFirstJointPitch the target pitch of the first joint
     * @param targetStringAngle     the string angle of the string
     * @param targetStringLength    the target string length of the climber
     */
    private void setTargetMechanism2dState(Rotation2d targetFirstJointPitch, Rotation2d targetStringAngle, double targetStringLength) {
        targetFirstJointLigament.setAngle(flipAngle(targetFirstJointPitch));
        targetStringPositionLigament.setAngle(flipAngle(targetStringAngle));
        targetStringPositionLigament.setLength(targetStringLength);
    }

    /**
     * Logs the mechanism, and the poses.
     *
     * @param currentPitch the current pitch or the climber
     */
    private void log(Rotation2d currentPitch) {
        Logger.recordOutput(key, mechanism);
        Logger.recordOutput("Poses/Components/" + name + "FirstJointPose", calculateClimberFirstJointPose(firstJointOriginPoint, currentPitch));
        Logger.recordOutput("Poses/Components/" + name + "secondJointPose", calculateClimberSecondJointPose(calculateClimberFirstJointPose(firstJointOriginPoint, currentPitch), currentState));
    }


    private Pose3d calculateClimberSecondJointPose(Pose3d firstJointPose, ClimberConstants.ClimberState currentState) {
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

    private Pose3d calculateClimberFirstJointPose(Translation3d originPoint, Rotation2d firstJointPitch) {
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

    private Rotation2d calculateClimberFirstJointPitch(double stringLength) {
        final double numeratorCalculation =
                Math.pow(ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS, 2)
                        + Math.pow(ClimberConstants.FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS, 2)
                        - Math.pow(stringLength, 2);
        final double denominatorCalculation = 2 * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS * ClimberConstants.FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS;
        final double division = numeratorCalculation / denominatorCalculation;
        final double angle = Math.acos(division) + ClimberConstants.FIRST_JOINT_ANGLE_ADDITION.getRadians();
        return Rotation2d.fromRadians(angle);
    }

    private double calculateStringLengthMeters(double position) {
        return RobotContainer.CLIMBER.toMeters(position) + ClimberConstants.STRING_LENGTH_ADDITION_METERS;
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
}
