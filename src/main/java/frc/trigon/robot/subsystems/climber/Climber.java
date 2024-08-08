package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private ClimberConstants.ClimberState currentState = ClimberConstants.ClimberState.RESTING;

    public Climber() {
        setName("Climber");
    }

    @Override
    public void periodic() {
        rightMotor.update();
        leftMotor.update();
        updateMechanism();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        rightMotorDrive(voltageMeasure);
        leftMotorDrive(voltageMeasure);
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("RightClimberMotor")
                .linearPosition(Units.Meters.of(toMeters(rightMotor.getSignal(TalonFXSignal.POSITION))))
                .linearVelocity(Units.MetersPerSecond.of(toMeters(rightMotor.getSignal(TalonFXSignal.VELOCITY))))
                .voltage(Units.Volts.of(rightMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
        log.motor("LeftClimberMotor")
                .linearPosition(Units.Meters.of(toMeters(leftMotor.getSignal(TalonFXSignal.POSITION))))
                .linearVelocity(Units.MetersPerSecond.of(toMeters(leftMotor.getSignal(TalonFXSignal.VELOCITY))))
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
        setTargetPosition(targetState.positionMeters, targetState.positionMeters);
    }

    void setTargetPosition(double targetRightPositionMeters, double targetLeftPositionMeters) {
        rightMotor.setControl(motionMagicRequest.withPosition(targetRightPositionMeters));
        leftMotor.setControl(motionMagicRequest.withPosition(targetLeftPositionMeters));
    }

    private void updateMechanism() {
        ClimberConstants.RIGHT_MECHANISM.update(
                toMeters(rightMotor.getSignal(TalonFXSignal.POSITION)),
                toMeters(rightMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
        ClimberConstants.LEFT_MECHANISM.update(
                toMeters(leftMotor.getSignal(TalonFXSignal.POSITION)),
                toMeters(leftMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );

        Logger.recordOutput("Poses/Components/RightClimberFirstJointPose", getRightClimberFirstJointPose());
        Logger.recordOutput("Poses/Components/LeftClimberFirstJointPose", getLeftClimberFirstJointPose());
        Logger.recordOutput("Poses/Components/RightClimberSecondJointPose", getRightClimberSecondJointPose());
        Logger.recordOutput("Poses/Components/LeftClimberSecondJointPose", getLeftClimberSecondJointPose());
    }

    private Pose3d getRightClimberSecondJointPose() {
        if (currentState != ClimberConstants.ClimberState.RESTING) {
            return new Pose3d(
                    ClimberConstants.RIGHT_CLIMBER_SECOND_JOINT_ORIGIN_POINT,
                    new Rotation3d(0, 90, 0)
            );
        }
        return new Pose3d(
                ClimberConstants.RIGHT_CLIMBER_SECOND_JOINT_ORIGIN_POINT,
                new Rotation3d()
        );
    }

    private Pose3d getLeftClimberSecondJointPose() {
        if (currentState != ClimberConstants.ClimberState.RESTING) {
            return new Pose3d(
                    ClimberConstants.LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT,
                    new Rotation3d(0, 90, 0)
            );
        }
        return new Pose3d(
                ClimberConstants.LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT,
                new Rotation3d()
        );
    }

    private Pose3d getRightClimberFirstJointPose() {
        final Pose3d currentFirstJointPose = new Pose3d(
                ClimberConstants.RIGHT_CLIMBER_FIRST_JOINT_ORIGIN_POINT,
                new Rotation3d(0, getRightClimberPitch().getRadians(), 0)
        );
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, ClimberConstants.MAXIMUM_HEIGHT_METERS, 0),
                new Rotation3d()
        );
        return currentFirstJointPose.transformBy(climberTransform);
    }

    private Pose3d getLeftClimberFirstJointPose() {
        final Pose3d currentFirstJointPose = new Pose3d(
                ClimberConstants.LEFT_CLIMBER_FIRST_JOINT_ORIGIN_POINT,
                new Rotation3d(0, getLeftClimberPitch().getRadians(), 0)
        );
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, ClimberConstants.MAXIMUM_HEIGHT_METERS, 0),
                new Rotation3d()
        );
        return currentFirstJointPose.transformBy(climberTransform);
    }

    private Rotation2d getRightClimberPitch() {
        final Translation3d difference = getRightClimberStringConnectionPointPose()
                .getTranslation()
                .minus(ClimberConstants.STRING_POSE.getTranslation()
                );
        final Rotation2d stringToClimberAngle = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
        final double climberPitchDegrees = 180 - ClimberConstants.STRING_PITCH.minus(stringToClimberAngle).getDegrees();
        return Rotation2d.fromDegrees(climberPitchDegrees);
    }

    private Rotation2d getLeftClimberPitch() {
        final Translation3d difference = getLeftClimberStringConnectionPointPose()
                .getTranslation()
                .minus(ClimberConstants.STRING_POSE.getTranslation()
                );
        final Rotation2d stringToClimberAngle = Rotation2d.fromRadians(Math.atan2(difference.getY(), difference.getX()));
        final double climberPitchDegrees = 180 - ClimberConstants.STRING_PITCH.minus(stringToClimberAngle).getDegrees();
        return Rotation2d.fromDegrees(climberPitchDegrees);
    }

    private Pose3d getRightClimberStringConnectionPointPose() {
        final double currentStringLength = toMeters(rightMotor.getSignal(TalonFXSignal.POSITION)) * ClimberConstants.STRING_LENGTH_METERS;
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, 0, currentStringLength),
                new Rotation3d()
        );
        return ClimberConstants.STRING_POSE.transformBy(climberTransform);
    }

    private Pose3d getLeftClimberStringConnectionPointPose() {
        final double currentStringLength = toMeters(leftMotor.getSignal(TalonFXSignal.POSITION)) * ClimberConstants.STRING_LENGTH_METERS;
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, 0, currentStringLength),
                new Rotation3d()
        );
        return ClimberConstants.STRING_POSE.transformBy(climberTransform);
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberConstants.DRUM_DIAMETER_METERS);
    }
}