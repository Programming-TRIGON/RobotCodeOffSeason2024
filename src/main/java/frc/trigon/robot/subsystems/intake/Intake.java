package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.constants.OperatorConstants;
import frc.trigon.robot.subsystems.MotorSubsystem;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor masterMotor = IntakeConstants.MASTER_MOTOR;
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(IntakeConstants.FOC_ENABLED);
    private final StaticBrake staticBrakeRequest = new StaticBrake();
    private IntakeConstants.IntakeState targetState;

    public Intake() {
        setName("Intake");
    }

    @Override
    public void updatePeriodically() {
        masterMotor.update();
        IntakeConstants.DISTANCE_SENSOR.updateSensor();
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
        targetState = IntakeConstants.IntakeState.STOP;
        IntakeConstants.MECHANISM.setTargetVelocity(0);
    }

    @Override
    public void updateMechanisms() {
        IntakeConstants.MECHANISM.update(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }

    public IntakeConstants.IntakeState getTargetState() {
        return targetState;
    }

    void sendStaticBrakeRequest() {
        masterMotor.setControl(staticBrakeRequest);
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
        return IntakeConstants.BOOLEAN_EVENT.getAsBoolean();
    }

    /**
     * Indicates to the driver that a note has been collected by rumbling the controller.
     */
    void indicateCollection() {
        OperatorConstants.DRIVER_CONTROLLER.rumble(IntakeConstants.RUMBLE_DURATION_SECONDS, IntakeConstants.RUMBLE_POWER);
    }
}