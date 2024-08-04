package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Shooter extends MotorSubsystem {
    private final TalonFXMotor
            rightMotor = ShooterConstants.RIGHT_MOTOR,
            leftMotor = ShooterConstants.LEFT_MOTOR;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(ShooterConstants.FOC_ENABLED);

    public Shooter() {
        setName("Shooter");
    }

    @Override
    public void setBrake(boolean brake) {
        rightMotor.setBrake(brake);
        leftMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    @Override
    public void periodic() {
        rightMotor.update();
        leftMotor.update();
    }

    void setTargetVelocity(double rightMotorTargetVelocityRotationsPerSecond, double leftMotorTargetVelocityRotationsPerSecond) {
        rightMotor.setControl(velocityRequest.withVelocity(rightMotorTargetVelocityRotationsPerSecond));
        leftMotor.setControl(velocityRequest.withVelocity(leftMotorTargetVelocityRotationsPerSecond));
    }

    private void updateMechanism() {
        ShooterConstants.RIGHT_MECHANISM.update(rightMotor.getSignal(TalonFXSignal.VELOCITY));
        ShooterConstants.LEFT_MECHANISM.update(leftMotor.getSignal(TalonFXSignal.VELOCITY));
    }
}
