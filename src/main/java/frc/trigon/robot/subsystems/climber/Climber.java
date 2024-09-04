package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.commands.factories.GeneralCommands;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

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
    private ClimberConstants.ClimberState targetState = ClimberConstants.ClimberState.REST;
    private boolean isClimbing = false;

    public Climber() {
        setName("Climber");
        GeneralCommands.getDelayedCommand(3, this::configureChangingDefaultCommand).schedule();
        configurePositionResettingTrigger(rightMotor);
        configurePositionResettingTrigger(leftMotor);
    }

    @Override
    public void periodic() {
        rightMotor.update();
        leftMotor.update();
        updateMechanisms();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        driveRightMotor(voltageMeasure.plus(Units.Volts.of(calculateStayingInPlaceFeedback(rightMotor.getSignal(TalonFXSignal.POSITION), ClimberConstants.SYSID_IS_ON_CHAIN))));
        driveLeftMotor(voltageMeasure.plus(Units.Volts.of(calculateStayingInPlaceFeedback(leftMotor.getSignal(TalonFXSignal.POSITION), ClimberConstants.SYSID_IS_ON_CHAIN))));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        final double rightMotorPosition = rightMotor.getSignal(TalonFXSignal.POSITION);
        final double leftMotorPosition = leftMotor.getSignal(TalonFXSignal.POSITION);

        log.motor("RightClimberMotor")
                .angularPosition(Units.Rotations.of(rightMotorPosition))
                .angularVelocity(Units.RotationsPerSecond.of(rightMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(rightMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE) - calculateStayingInPlaceFeedback(rightMotorPosition, ClimberConstants.SYSID_IS_ON_CHAIN)));
        log.motor("LeftClimberMotor")
                .angularPosition(Units.Rotations.of(leftMotorPosition))
                .angularVelocity(Units.RotationsPerSecond.of(leftMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(leftMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE) - calculateStayingInPlaceFeedback(leftMotorPosition, ClimberConstants.SYSID_IS_ON_CHAIN)));
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
        return atTargetRightState() && atTargetLeftState();
    }

    public boolean atTargetRightState() {
        return Math.abs(rightMotor.getSignal(TalonFXSignal.POSITION) - targetState.positionRotations) < ClimberConstants.CLIMBER_TOLERANCE_ROTATIONS;
    }

    public boolean atTargetLeftState() {
        return Math.abs(leftMotor.getSignal(TalonFXSignal.POSITION) - targetState.positionRotations) < ClimberConstants.CLIMBER_TOLERANCE_ROTATIONS;
    }

    public void setIsClimbing(boolean isClimbing) {
        this.isClimbing = isClimbing;
    }

    public boolean isClimbing() {
        return isClimbing;
    }

    void driveRightMotor(Measure<Voltage> voltageMeasure) {
        rightMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    void driveLeftMotor(Measure<Voltage> voltageMeasure) {
        leftMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        this.targetState = targetState;
        setTargetPosition(targetState.positionRotations, targetState.positionRotations, targetState.affectedByRobotWeight);
    }

    void setTargetPosition(double targetRightPositionRotations, double targetLeftPositionRotations, boolean affectedByRobotWeight) {
        rightMotor.setControl(determineRequest(affectedByRobotWeight)
                .withPosition(targetRightPositionRotations)
                .withFeedForward(calculateStayingInPlaceFeedback(rightMotor.getSignal(TalonFXSignal.POSITION), affectedByRobotWeight)));
        leftMotor.setControl(determineRequest(affectedByRobotWeight)
                .withPosition(targetLeftPositionRotations)
                .withFeedForward(calculateStayingInPlaceFeedback(leftMotor.getSignal(TalonFXSignal.POSITION), affectedByRobotWeight)));
    }

    private void configurePositionResettingTrigger(TalonFXMotor motor) {
        final Trigger positionResettingTrigger = new Trigger(() -> hasHitReverseLimit(motor)).debounce(ClimberConstants.LIMIT_SWITCH_DEBOUNCE_TIME_SECONDS);
        positionResettingTrigger.onFalse(new InstantCommand(() -> motor.setPosition(ClimberConstants.LIMIT_SWITCH_PRESSED_POSITION)));
    }

    private boolean hasHitReverseLimit(TalonFXMotor motor) {
        return motor.getSignal(TalonFXSignal.REVERSE_LIMIT) == 0;
    }

    private DynamicMotionMagicVoltage determineRequest(boolean affectedByRobotWeight) {
        return affectedByRobotWeight ? onChainPositionRequest : groundedPositionRequest;
    }

    private double calculateStayingInPlaceFeedback(double positionRotations, boolean affectedByRobotWeight) {
        return affectedByRobotWeight ?
                calculateParabola(positionRotations, ClimberConstants.ON_CHAIN_A, ClimberConstants.ON_CHAIN_B, ClimberConstants.ON_CHAIN_C) :
                calculateParabola(positionRotations, ClimberConstants.GROUNDED_A, ClimberConstants.GROUNDED_B, ClimberConstants.GROUNDED_C);
    }

    private double calculateParabola(double x, double a, double b, double c) {
        return a * Math.pow(x, 2) + b * x + c;
    }

    private void configureChangingDefaultCommand() {
        final Trigger climbingTrigger = new Trigger(() -> isClimbing);
        climbingTrigger.onTrue(new InstantCommand(this::defaultToClimbing));
        climbingTrigger.onFalse(new InstantCommand(this::defaultToResting));
    }

    private void defaultToResting() {
        changeDefaultCommand(ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.REST));
    }

    private void defaultToClimbing() {
        changeDefaultCommand(ClimberCommands.getSetTargetStateCommand(ClimberConstants.ClimberState.CLIMB));
    }

    private void updateMechanisms() {
        ClimberConstants.RIGHT_VISUALIZATION.update(targetState, rightMotor.getSignal(TalonFXSignal.POSITION), rightMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
        ClimberConstants.LEFT_VISUALIZATION.update(targetState, leftMotor.getSignal(TalonFXSignal.POSITION), leftMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE));
    }
}