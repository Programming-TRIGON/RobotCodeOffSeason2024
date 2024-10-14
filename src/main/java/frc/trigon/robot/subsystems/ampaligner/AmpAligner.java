package frc.trigon.robot.subsystems.ampaligner;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.subsystems.pitcher.PitcherConstants;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class AmpAligner extends MotorSubsystem {
    private final TalonFXMotor motor = AmpAlignerConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(AmpAlignerConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(AmpAlignerConstants.FOC_ENABLED);
    private AmpAlignerConstants.AmpAlignerState targetState = AmpAlignerConstants.AmpAlignerState.CLOSE;
    private Rotation2d targetAngle = Rotation2d.fromDegrees(0);

    public AmpAligner() {
        setName("AmpAligner");
        configurePositionResettingTrigger();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void updatePeriodically() {
        motor.update();
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        motor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("AmpAligner")
                .angularPosition(Units.Rotations.of(motor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return AmpAlignerConstants.SYSID_CONFIG;
    }

    @Override
    public void updateMechanism() {
        PitcherConstants.PITCHER_AND_AMP_ALIGNER_MECHANISM.setSecondJointTargetAngle(Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)));
        PitcherConstants.PITCHER_AND_AMP_ALIGNER_MECHANISM.setSecondJointCurrentAngle(getCurrentAngle());
    }

    public AmpAlignerConstants.AmpAlignerState getTargetState() {
        return targetState;
    }

    /**
     * If the pitcher closes before the amp aligner is closed, the amp aligner hits the amp.
     * This function should be used to ensure that the pitcher closes after the amp is closed enough.
     *
     * @return if the pitcher is ready to close
     */
    public boolean isReadyForDefaultPitcherMovement() {
        return getCurrentAngle().getRotations() > AmpAlignerConstants.READY_FOR_DEFAULT_PITCHER_MOVEMENT_ANGLE.getRotations();
    }

    public boolean atTargetState() {
        return atAngle(targetState.targetAngle);
    }

    public boolean atAngle(Rotation2d targetAngle) {
        return Math.abs(getCurrentAngle().getDegrees() - targetAngle.getDegrees()) < AmpAlignerConstants.ANGLE_TOLERANCE.getDegrees();
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION));
    }

    void setTargetState(AmpAlignerConstants.AmpAlignerState targetState) {
        this.targetState = targetState;
        setTargetAngle(targetState.targetAngle);
    }

    void setTargetAngle(Rotation2d targetAngle) {
        this.targetAngle = targetAngle;
        motor.setControl(positionRequest
                .withPosition(targetAngle.getRotations())
                .withFeedForward(calculateKGOutput())
        );
    }

    private void configurePositionResettingTrigger() {
        final Trigger hitLimitSwitchTrigger = new Trigger(this::hasHitForwardLimit).debounce(AmpAlignerConstants.LIMIT_SWITCH_DEBOUNCE_TIME_SECONDS);
        hitLimitSwitchTrigger.onTrue(new InstantCommand(() -> motor.setPosition(AmpAlignerConstants.LIMIT_SWITCH_PRESSED_ANGLE.getRotations())).ignoringDisable(true));
    }

    private boolean hasHitForwardLimit() {
        return motor.getSignal(TalonFXSignal.FORWARD_LIMIT) == 0;
    }

    private double calculateKGOutput() {
        if (!RobotHardwareStats.isSimulation())
            return AmpAlignerConstants.KG * Math.cos(RobotContainer.PITCHER.getCurrentPitch().getRadians() + this.getCurrentAngle().getRadians());
        return 0;
    }
}
