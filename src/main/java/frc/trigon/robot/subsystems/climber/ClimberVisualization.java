package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

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
    private Rotation2d
            climberCurrentPitch,
            climberTargetPitch;
    private final TalonFXMotor motor;
    private ClimberConstants.ClimberState currentState;

    /**
     * Constructs a ClimberVisualization object.
     *
     * @param name                  the name of the mechanism
     * @param firstJointColor       the color of the first joint of the climber in the mechanism
     * @param stringColor           the color of the string in the mechanism
     * @param firstJointOriginPoint the first joint origin point
     * @param motor                 the motor
     */
    public ClimberVisualization(String name, Color8Bit firstJointColor, Color8Bit stringColor, Translation3d firstJointOriginPoint, TalonFXMotor motor) {
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

        this.motor = motor;
        this.climberCurrentPitch = getClimberFirstJointPitch(getStringLengthMeters(motor, TalonFXSignal.POSITION));
        this.climberTargetPitch = getClimberFirstJointPitch(getStringLengthMeters(motor, TalonFXSignal.CLOSED_LOOP_REFERENCE));
        this.currentState = ClimberConstants.ClimberState.RESTING;
    }

    /**
     * Updates the current climber arm angle and the string angle, then logs the mechanism.
     *
     * @param currentState the current climber state
     */
    public void update(ClimberConstants.ClimberState currentState) {
        this.currentState = currentState;
        currentFirstJointLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - climberCurrentPitch.getDegrees());
        currentStringPositionLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - calculateStringAngle(climberCurrentPitch.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION), getStringLengthMeters(motor, TalonFXSignal.POSITION)).getDegrees());
        currentStringPositionLigament.setLength(getStringLengthMeters(motor, TalonFXSignal.POSITION));
        setTargetPosition();
        update();
    }

    /**
     * Logs the mechanism.
     */
    public void update() {
        climberCurrentPitch = getClimberFirstJointPitch(getStringLengthMeters(motor, TalonFXSignal.POSITION));
        climberTargetPitch = getClimberFirstJointPitch(getStringLengthMeters(motor, TalonFXSignal.CLOSED_LOOP_REFERENCE));
        Logger.recordOutput(key, mechanism);
        Logger.recordOutput("Poses/Components/" + name + "FirstJointPose", climberCurrentPitch);
        Logger.recordOutput("Poses/Components/" + name + "secondJointPose", getClimberSecondJointPose(getClimberFirstJointPose(firstJointOriginPoint, climberCurrentPitch), currentState));
    }

    /**
     * Sets the target climber arm angle and the target string angle but doesn't log the mechanism.
     */
    public void setTargetPosition() {
        targetFirstJointLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - climberTargetPitch.getDegrees());
        targetStringPositionLigament.setAngle(ClimberConstants.MECHANISM_STARTING_ANGLE - calculateStringAngle(climberTargetPitch.minus(ClimberConstants.FIRST_JOINT_ANGLE_ADDITION), getStringLengthMeters(motor, TalonFXSignal.CLOSED_LOOP_REFERENCE)).getDegrees());
        targetStringPositionLigament.setLength(getStringLengthMeters(motor, TalonFXSignal.CLOSED_LOOP_REFERENCE));
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

    private double getStringLengthMeters(TalonFXMotor motor, TalonFXSignal signal) {
        return toMeters(motor.getSignal(signal)) + ClimberConstants.STRING_LENGTH_ADDITION_METERS;
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberConstants.DRUM_DIAMETER_METERS);
    }
}
