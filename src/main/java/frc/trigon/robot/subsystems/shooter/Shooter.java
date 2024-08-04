package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.ShootingCalculations;

public class Shooter extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor
            rightMotor = ShooterConstants.RIGHT_MOTOR,
            leftMotor = ShooterConstants.LEFT_MOTOR;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(ShooterConstants.FOC_ENABLED);
    private double
            rightMotorTargetVelocityRotationsPerSecond = 0,
            leftMotorTargetVelocityRotationsPerSecond = 0;

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
        updateMechanism();
    }

    void reachTargetShootingVelocityFromShootingCalculations() {
        rightMotorTargetVelocityRotationsPerSecond = shootingCalculations.getTargetShootingState().targetShootingVelocityRotationsPerSecond();
        leftMotorTargetVelocityRotationsPerSecond = shootingCalculations.getTargetShootingState().targetShootingVelocityRotationsPerSecond();
        setTargetVelocity(rightMotorTargetVelocityRotationsPerSecond, leftMotorTargetVelocityRotationsPerSecond);
    }

    void setTargetVelocity(double rightMotorTargetVelocityRotationsPerSecond, double leftMotorTargetVelocityRotationsPerSecond) {
        setRightTargetVelocity(rightMotorTargetVelocityRotationsPerSecond);
        setLeftTargetVelocity(leftMotorTargetVelocityRotationsPerSecond);
    }

    void setRightTargetVelocity(double targetVelocityRotationsPerSecond) {
        rightMotor.setControl(velocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    void setLeftTargetVelocity(double targetVelocityRotationsPerSecond) {
        leftMotor.setControl(velocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    private void updateMechanism() {
        ShooterConstants.RIGHT_MECHANISM.update(rightMotor.getSignal(TalonFXSignal.VELOCITY), rightMotorTargetVelocityRotationsPerSecond);
        ShooterConstants.LEFT_MECHANISM.update(leftMotor.getSignal(TalonFXSignal.VELOCITY), leftMotorTargetVelocityRotationsPerSecond);
    }
}
