package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.MotorSubsystem;

public class Climber extends MotorSubsystem {
    private final TalonFXMotor
            rightMotor = ClimberConstants.RIGHT_MOTOR,
            leftMotor = ClimberConstants.LEFT_MOTOR;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ClimberConstants.ENABLE_FOC);

    public Climber() {
        setName("Climber");
    }

    @Override
    public void periodic() {
        rightMotor.update();
        leftMotor.update();
        updateMechanism();
    }

    public void drive(Measure<Voltage> voltageMeasure) {
        rightMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
        leftMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

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

    public SysIdRoutine.Config getSysIdConfig() {
        return ClimberConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    void setTargetState(ClimberConstants.ClimberState targetRightState, ClimberConstants.ClimberState targetLeftState) {
        setTargetPosition(targetRightState.positionMeters, targetLeftState.positionMeters);
    }

    void setTargetPosition(double targetRightPositionMeters, double targetLeftPositionMeters) {
        setRightMotorTargetPosition(targetRightPositionMeters);
        setLeftMotorTargetPosition(targetLeftPositionMeters);
    }

    void setRightMotorTargetPosition(double targetPositionMeters) {
        rightMotor.setControl(motionMagicRequest.withPosition(targetPositionMeters));
    }

    void setLeftMotorTargetPosition(double targetPositionMeters) {
        leftMotor.setControl(motionMagicRequest.withPosition(targetPositionMeters));
    }

    private void updateMechanism() {
        ClimberConstants.RIGHT_MECHANISM.update(
                toMeters(rightMotor.getSignal(TalonFXSignal.POSITION)),
                toMeters(rightMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)));
        ClimberConstants.LEFT_MECHANISM.update(
                toMeters(leftMotor.getSignal(TalonFXSignal.POSITION)),
                toMeters(leftMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)));

        Logger.recordOutput("Poses/Components/RightClimberPose", getRightClimberPose());
        Logger.recordOutput("Poses/Components/LeftClimberPose", getLeftClimberPose());
    }

    private Pose3d getRightClimberPose() {
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, 0, toMeters(rightMotor.getSignal(TalonFXSignal.POSITION))),
                new Rotation3d()
        );
        return ClimberConstants.RIGHT_CLIMBER_ORIGIN_POINT.transformBy(climberTransform);
    }

    private Pose3d getLeftClimberPose() {
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, 0, toMeters(leftMotor.getSignal(TalonFXSignal.POSITION))),
                new Rotation3d()
        );
        return ClimberConstants.LEFT_CLIMBER_ORIGIN_POINT.transformBy(climberTransform);
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberConstants.DRUM_DIAMETER_METERS);
    }
}
