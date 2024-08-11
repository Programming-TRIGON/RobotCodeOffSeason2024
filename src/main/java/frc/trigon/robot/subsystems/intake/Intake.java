package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Intake extends MotorSubsystem {
    private IntakeConstants.IntakeState targetState;
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
        IntakeConstants.MECHANISM.setTargetVelocity(0);
    }

    public IntakeConstants.IntakeState getTargetState() {
        return targetState;
    }

    void setTargetState(IntakeConstants.IntakeState targetState) {
        this.targetState = targetState;
        setTargetVoltage(targetState.voltage);
    }

    void setTargetVoltage(double targetVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(targetVoltage));
        IntakeConstants.MECHANISM.setTargetVelocity(targetVoltage);
    }

    boolean hasNote() {
        if (IntakeConstants.BOOLEAN_EVENT.debounce(IntakeConstants.NOTE_DETECTION_CONFIRMATION_DELAY_SECONDS).getAsBoolean())
            setBrake(true);
        return IntakeConstants.DISTANCE_SENSOR.getScaledValue() < IntakeConstants.NOTE_DISTANCE_THRESHOLD_METERS;
    }

    /**
     * Rumbles the driver controller.
     */
    void indicateCollection() {
        OperatorConstants.DRIVER_CONTROLLER.rumble(IntakeConstants.RUMBLE_DURATION_SECONDS, IntakeConstants.RUMBLE_POWER);
    }

    private void updateMechanism() {
        IntakeConstants.MECHANISM.update(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }
}