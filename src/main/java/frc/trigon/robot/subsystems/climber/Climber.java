package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
            groundedPositionRequest = new DynamicMotionMagicVoltage(
            0,
            ClimberConstants.MAX_GROUNDED_VELOCITY,
            ClimberConstants.MAX_GROUNDED_ACCELERATION,
            0
    ).withSlot(ClimberConstants.GROUNDED_SLOT).withEnableFOC(ClimberConstants.ENABLE_FOC),
            onChainPositionRequest = new DynamicMotionMagicVoltage(
                    0,
                    ClimberConstants.MAX_ON_CHAIN_VELOCITY,
                    ClimberConstants.MAX_ON_CHAIN_ACCELERATION,
                    0
            ).withSlot(ClimberConstants.ON_CHAIN_SLOT).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private ClimberConstants.ClimberState targetState = ClimberConstants.ClimberState.RESTING;

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
        driveRightMotor(voltageMeasure.plus(Units.Volts.of(calculateStayingInPlaceVoltage(rightMotor.getSignal(TalonFXSignal.POSITION), targetState.affectedByRobotWeight))));
        driveLeftMotor(voltageMeasure.plus(Units.Volts.of(calculateStayingInPlaceVoltage(leftMotor.getSignal(TalonFXSignal.POSITION), targetState.affectedByRobotWeight))));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("RightClimberMotor")
                .linearPosition(Units.Meters.of(rightMotor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(rightMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(rightMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE) - calculateStayingInPlaceVoltage(rightMotor.getSignal(TalonFXSignal.POSITION), targetState.affectedByRobotWeight)));
        log.motor("LeftClimberMotor")
                .linearPosition(Units.Meters.of(leftMotor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(leftMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(leftMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE) - calculateStayingInPlaceVoltage(leftMotor.getSignal(TalonFXSignal.POSITION), targetState.affectedByRobotWeight)));
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

    public boolean atTargetState() {
        return Math.abs(toMeters(rightMotor.getSignal(TalonFXSignal.POSITION)) - targetState.positionMeters) < ClimberConstants.CLIMBER_TOLERANCE_METERS &&
                Math.abs(toMeters(leftMotor.getSignal(TalonFXSignal.POSITION)) - targetState.positionMeters) < ClimberConstants.CLIMBER_TOLERANCE_METERS;
    }

    void driveRightMotor(Measure<Voltage> voltageMeasure) {
        rightMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    void driveLeftMotor(Measure<Voltage> voltageMeasure) {
        leftMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        this.targetState = targetState;
        setTargetPosition(targetState.positionMeters, targetState.positionMeters, targetState.affectedByRobotWeight);
    }

    void setTargetPosition(double targetRightPositionMeters, double targetLeftPositionMeters, boolean affectedByRobotWeight) {
        rightMotor.setControl(determineRequest(affectedByRobotWeight)
                .withPosition(targetRightPositionMeters)
                .withFeedForward(calculateStayingInPlaceVoltage(rightMotor.getSignal(TalonFXSignal.POSITION), affectedByRobotWeight)));
        leftMotor.setControl(determineRequest(affectedByRobotWeight)
                .withPosition(targetLeftPositionMeters)
                .withFeedForward(calculateStayingInPlaceVoltage(leftMotor.getSignal(TalonFXSignal.POSITION), affectedByRobotWeight)));
    }

    private DynamicMotionMagicVoltage determineRequest(boolean affectedByRobotWeight) {
        return affectedByRobotWeight ? onChainPositionRequest : groundedPositionRequest;
    }

    private double calculateStayingInPlaceVoltage(double positionRotations, boolean affectedByRobotWeight) {
        return affectedByRobotWeight ?
                calculateParabola(positionRotations, ClimberConstants.ON_CHAIN_A, ClimberConstants.ON_CHAIN_B, ClimberConstants.ON_CHAIN_C)
                : calculateParabola(positionRotations, ClimberConstants.GROUNDED_A, ClimberConstants.GROUNDED_B, ClimberConstants.GROUNDED_C);
    }

    private double calculateParabola(double x, double a, double b, double c) {
        return a * Math.pow(x, 2) + b * x + c;
    }

    private void updateMechanisms() {
        ClimberConstants.RIGHT_MECHANISM.update(targetState, rightMotor.getSignal(TalonFXSignal.POSITION), rightMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
        ClimberConstants.LEFT_MECHANISM.update(targetState, leftMotor.getSignal(TalonFXSignal.POSITION), leftMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
    }

    double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberConstants.DRUM_DIAMETER_METERS);
    }
}