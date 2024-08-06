package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.MotorSubsystem;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor
            masterMotor = IntakeConstants.MASTER_MOTOR,
            followerMotor = IntakeConstants.FOLLOWER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);

    public Intake() {
        setName("Intake");
    }

    @Override
    public void periodic() {
        masterMotor.update();
        IntakeConstants.DISTANCE_SENSOR.updateSensor();
        updateMechanism();
    }

    @Override
    public void setBrake(boolean brake) {
        masterMotor.setBrake(brake);
        followerMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
        IntakeConstants.MECHANISM.setTargetVelocity(targetVoltage);
    }

    boolean hasNote() {
        return IntakeConstants.DISTANCE_SENSOR.getScaledValue() < IntakeConstants.NOTE_DISTANCE_THRESHOLD_METERS;
    }

    private void updateMechanism() {
        IntakeConstants.MECHANISM.update(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }
}
