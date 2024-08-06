package frc.trigon.robot.subsystems.ampaligner;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class AmpAligner extends MotorSubsystem {
    private final TalonFXMotor motor = AmpAlignerConstants.MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(AmpAlignerConstants.FOC_ENABLED);

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        motor.update();
        updateMechanism();
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
        log.motor("Pitcher")
                .linearPosition(Units.Meters.of(motor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(motor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return AmpAlignerConstants.SYSID_CONFIG;
    }

    void openAmpAligner() {
        motor.setControl(voltageRequest.withOutput(AmpAlignerConstants.AmpAlignerState.OPENING.voltage));
    }

    void closeAmpAligner() {
        motor.setControl(voltageRequest.withOutput(AmpAlignerConstants.AmpAlignerState.CLOSING.voltage));
    }

    boolean isForwardLimitSwitchPressed() {
        return AmpAlignerConstants.FORWARD_LIMIT_SWITCH.getBinaryValue();
    }

    boolean isBackwardLimitSwitchPressed() {
        return AmpAlignerConstants.BACKWARD_LIMIT_SWITCH.getBinaryValue();
    }

    private static void updateMechanism() {
        AmpAlignerConstants.MECHANISM.update();
    }
}
