package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
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
    private Rotation2d
            rightClimberCurrentPitch,
            rightClimberTargetPitch,
            leftClimberCurrentPitch,
            leftClimberTargetPitch;

    public Climber() {
        setName("Climber");
    }

    @Override
    public void periodic() {
        rightMotor.update();
        leftMotor.update();
        rightClimberCurrentPitch = getClimberFirstJointPitch(getStringLengthMeters(rightMotor, TalonFXSignal.POSITION));
        rightClimberTargetPitch = getClimberFirstJointPitch(getStringLengthMeters(rightMotor, TalonFXSignal.CLOSED_LOOP_REFERENCE));
        leftClimberCurrentPitch = getClimberFirstJointPitch(getStringLengthMeters(leftMotor, TalonFXSignal.POSITION));
        leftClimberTargetPitch = getClimberFirstJointPitch(getStringLengthMeters(leftMotor, TalonFXSignal.CLOSED_LOOP_REFERENCE));
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
                .withFeedForward(calculateFeedforward(rightClimberCurrentPitch, affectedByRobotWeight)));
        leftMotor.setControl(determineRequest(affectedByRobotWeight)
                .withPosition(targetLeftPositionMeters)
                .withFeedForward(calculateFeedforward(leftClimberCurrentPitch, affectedByRobotWeight)));
    }

    private DynamicMotionMagicVoltage determineRequest(boolean affectedByRobotWeight) {
        return affectedByRobotWeight ? climbingPositionRequest : nonClimbingPositionRequest;
    }

    private double calculateFeedforward(Rotation2d firstJointPitch, boolean affectedByRobotWeight) {
        return affectedByRobotWeight ?
                ClimberConstants.ON_CHAIN_KG * Math.cos(firstJointPitch.getRadians()) :
                ClimberConstants.GROUNDED_KG * Math.cos(firstJointPitch.getRadians());
    }

    private void updateMechanisms() {
        ClimberConstants.RIGHT_MECHANISM.update(
                rightClimberCurrentPitch,
                rightClimberTargetPitch,
                getStringLengthMeters(rightMotor, TalonFXSignal.POSITION),
                getStringLengthMeters(rightMotor, TalonFXSignal.CLOSED_LOOP_REFERENCE),
                currentState
        );
        ClimberConstants.LEFT_MECHANISM.update(
                leftClimberCurrentPitch,
                leftClimberTargetPitch,
                getStringLengthMeters(leftMotor, TalonFXSignal.POSITION),
                getStringLengthMeters(leftMotor, TalonFXSignal.CLOSED_LOOP_REFERENCE),
                currentState
        );
    }

    Rotation2d getClimberFirstJointPitch(double stringLength) {
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