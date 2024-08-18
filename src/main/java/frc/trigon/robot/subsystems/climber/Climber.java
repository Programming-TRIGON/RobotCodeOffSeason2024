package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;

public class Climber extends MotorSubsystem {
    private final TalonFXMotor
            rightMotor = ClimberConstants.RIGHT_MOTOR,
            leftMotor = ClimberConstants.LEFT_MOTOR;
    private final DynamicMotionMagicVoltage
            nonClimbingPositionRequest = new DynamicMotionMagicVoltage(
            0,
            ClimberConstants.MAX_NON_CLIMBING_VELOCITY,
            ClimberConstants.MAX_NON_CLIMBING_ACCELERATION,
            0
    ).withSlot(ClimberConstants.GROUNDED_SLOT).withEnableFOC(ClimberConstants.ENABLE_FOC),
            climbingPositionRequest = new DynamicMotionMagicVoltage(
                    0,
                    ClimberConstants.MAX_CLIMBING_VELOCITY,
                    ClimberConstants.MAX_CLIMBING_ACCELERATION,
                    0
            ).withSlot(ClimberConstants.ON_CHAIN_SLOT).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private ClimberConstants.ClimberState currentState = ClimberConstants.ClimberState.RESTING;

    public Climber() {
        setName("Climber");
    }

    @Override
    public void periodic() {
        rightMotor.update();
        leftMotor.update();
        updateMechanisms();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        rightMotorDrive(voltageMeasure);
        leftMotorDrive(voltageMeasure);
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("RightClimberMotor")
                .linearPosition(Units.Meters.of(rightMotor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(rightMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(rightMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
        log.motor("LeftClimberMotor")
                .linearPosition(Units.Meters.of(leftMotor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(leftMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(leftMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void setBrake(boolean brake) {
        rightMotor.setBrake(brake);
        leftMotor.setBrake(brake);
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ClimberConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    void rightMotorDrive(Measure<Voltage> voltageMeasure) {
        rightMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    void leftMotorDrive(Measure<Voltage> voltageMeasure) {
        leftMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        currentState = targetState;
        setTargetPosition(targetState.positionMeters, targetState.positionMeters, targetState.affectedByRobotWeight);
    }

    void setTargetPosition(double targetRightPositionMeters, double targetLeftPositionMeters, boolean affectedByRobotWeight) {
        rightMotor.setControl(determineRequest(affectedByRobotWeight)
                .withPosition(targetRightPositionMeters)
                .withFeedForward(calculateFeedforward(rightMotor, affectedByRobotWeight)));
        leftMotor.setControl(determineRequest(affectedByRobotWeight)
                .withPosition(targetLeftPositionMeters)
                .withFeedForward(calculateFeedforward(leftMotor, affectedByRobotWeight)));
    }

    private DynamicMotionMagicVoltage determineRequest(boolean affectedByRobotWeight) {
        return affectedByRobotWeight ? climbingPositionRequest : nonClimbingPositionRequest;
    }

    private double calculateFeedforward(TalonFXMotor motor, boolean affectedByRobotWeight) {
        return affectedByRobotWeight ?
                ClimberConstants.ON_CHAIN_KG * Math.cos(getClimberFirstJointPitch(motor).getRadians()) :
                ClimberConstants.GROUNDED_KG * Math.cos(getClimberFirstJointPitch(motor).getRadians());
    }

    private void updateMechanisms() {
        ClimberConstants.RIGHT_MECHANISM.update(
                getClimberFirstJointPitch(rightMotor),
                Rotation2d.fromRotations(rightMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)),
                getStringLengthMeters(rightMotor),
                toMeters(rightMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
        ClimberConstants.LEFT_MECHANISM.update(
                getClimberFirstJointPitch(leftMotor),
                Rotation2d.fromRotations(leftMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)),
                getStringLengthMeters(leftMotor),
                toMeters(leftMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        Logger.recordOutput("Poses/Components/RightClimberFirstJointPose", getClimberFirstJointPose(ClimberConstants.RIGHT_CLIMBER_FIRST_JOINT_ORIGIN_POINT, rightMotor));
        Logger.recordOutput("Poses/Components/LeftClimberFirstJointPose", getClimberFirstJointPose(ClimberConstants.LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT, leftMotor));
        Logger.recordOutput("Poses/Components/RightClimberSecondJointPose", getClimberSecondJointPose(getClimberFirstJointPose(ClimberConstants.RIGHT_CLIMBER_FIRST_JOINT_ORIGIN_POINT, rightMotor)));
        Logger.recordOutput("Poses/Components/LeftClimberSecondJointPose", getClimberSecondJointPose(getClimberFirstJointPose(ClimberConstants.LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT, leftMotor)));
    }

    private Pose3d getClimberSecondJointPose(Pose3d firstJointPose) {
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

    private Pose3d getClimberFirstJointPose(Translation3d originPoint, TalonFXMotor motor) {
        return new Pose3d(
                originPoint,
                new Rotation3d(0, getClimberFirstJointPitch(motor).getRadians(), 0)
        );
    }

    Rotation2d getClimberFirstJointPitch(TalonFXMotor motor) {
        final double numeratorCalculation =
                Math.pow(ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS, 2)
                        + Math.pow(ClimberConstants.FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS, 2)
                        - Math.pow(toMeters(getStringLengthMeters(motor)), 2);
        final double denominatorCalculation = 2 * ClimberConstants.FIRST_JOINT_POSE_TO_STRING_CONNECTION_DISTANCE_METERS * ClimberConstants.FIRST_JOINT_POSE_TO_DRUM_DISTANCE_METERS;
        final double division = numeratorCalculation / denominatorCalculation;
        final double angle = Math.acos(division + edu.wpi.first.math.util.Units.degreesToRadians(ClimberConstants.ANGLE_ADDITION_DEGREES));
        return Rotation2d.fromRadians(angle);
    }

    private double getStringLengthMeters(TalonFXMotor motor) {
        return toMeters(motor.getSignal(TalonFXSignal.POSITION)) + ClimberConstants.STRING_LENGTH_ADDITION_METERS;
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberConstants.DRUM_DIAMETER_METERS);
    }
}