package frc.trigon.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

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
        followerMotor.update();
        IntakeConstants.DISTANCE_SENSOR.updateSensor();
        updateMechanism();
    }

    @Override
    public void setBrake(boolean brake) {
        masterMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
        followerMotor.stopMotor();
    }

    void setTargetVoltage(double collectionVoltage) {
        masterMotor.setControl(voltageRequest.withOutput(collectionVoltage));
        IntakeConstants.MECHANISM.setTargetVelocity(collectionVoltage);
    }

    private double getDistance(double maximumDistance) {
        return IntakeConstants.DISTANCE_SENSOR.getDutyCycleValue(maximumDistance);
    }

    private void updateMechanism() {
        IntakeConstants.MECHANISM.update(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE));
    }
}
