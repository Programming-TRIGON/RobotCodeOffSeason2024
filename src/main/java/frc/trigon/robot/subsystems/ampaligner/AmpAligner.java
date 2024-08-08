package frc.trigon.robot.subsystems.ampaligner;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class AmpAligner extends MotorSubsystem {
    private final TalonFXMotor motor = AmpAlignerConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(AmpAlignerConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(AmpAlignerConstants.FOC_ENABLED);
    private AmpAlignerConstants.AmpAlignerState targetState = AmpAlignerConstants.AmpAlignerState.CLOSED;
    private boolean isStopped = true;

    @Override
    public void stop() {
        isStopped = true;
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        motor.update();
        updateMechanism();
        if (!isStopped)
            motor.setControl(positionRequest.withFeedForward(calculateFeedForward()));
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

    public AmpAlignerConstants.AmpAlignerState getTargetState() {
        return targetState;
    }

    void openAmpAligner() {
        isStopped = false;
        targetState = AmpAlignerConstants.AmpAlignerState.OPENED;
        motor.setControl(positionRequest
                .withPosition(targetState.targetPosition.getRotations())
                .withFeedForward(calculateFeedForward())
        );
    }

    void closeAmpAligner() {
        isStopped = false;
        targetState = AmpAlignerConstants.AmpAlignerState.CLOSED;
        motor.setControl(positionRequest
                .withPosition(targetState.targetPosition.getRotations())
                .withFeedForward(calculateFeedForward())
        );
    }

    private double calculateFeedForward() {
        return AmpAlignerConstants.KG * Math.cos(RobotContainer.PITCHER.getCurrentPitch().getRadians() + this.getCurrentAngle().getRadians());
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION));
    }

    private void updateMechanism() {
        AmpAlignerConstants.MECHANISM.update(Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION)), targetState.targetPosition);
    }
}
