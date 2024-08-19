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
                .withFeedForward(calculateFeedforward(rightMotor)));
        leftMotor.setControl(determineRequest(affectedByRobotWeight)
                .withPosition(targetLeftPositionMeters)
                .withFeedForward(calculateFeedforward(leftMotor)));
    }

    private DynamicMotionMagicVoltage determineRequest(boolean affectedByRobotWeight) {
        return affectedByRobotWeight ? climbingPositionRequest : nonClimbingPositionRequest;
    }

    private double calculateFeedforward(TalonFXMotor motor) {
        return ClimberConstants.A * Math.pow(motor.getSignal(TalonFXSignal.POSITION), 2) + ClimberConstants.B * motor.getSignal(TalonFXSignal.POSITION) + ClimberConstants.C;
    }

    private void updateMechanisms() {
        ClimberConstants.RIGHT_MECHANISM.update(currentState);
        ClimberConstants.LEFT_MECHANISM.update(currentState);
    }
}