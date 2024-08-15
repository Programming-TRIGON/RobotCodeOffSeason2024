package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Shooter extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor
            rightMotor = ShooterConstants.RIGHT_MOTOR,
            leftMotor = ShooterConstants.LEFT_MOTOR;
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ShooterConstants.FOC_ENABLED);

    public Shooter() {
        setName("Shooter");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("RightShooter")
                .angularPosition(Units.Rotations.of(rightMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(rightMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(rightMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
        log.motor("LeftShooter")
                .angularPosition(Units.Rotations.of(leftMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(leftMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(leftMotor.getSignal(TalonFXSignal.TORQUE_CURRENT)));
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
        ShooterConstants.RIGHT_MECHANISM.setTargetVelocity(0);
        ShooterConstants.LEFT_MECHANISM.setTargetVelocity(0);
    }

    @Override
    public void periodic() {
        rightMotor.update();
        leftMotor.update();
        updateMechanism();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        rightMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
        leftMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ShooterConstants.SYSID_CONFIG;
    }

    void reachTargetShootingVelocityFromShootingCalculations() {
        final double targetRightVelocityRotationsPerSecond = shootingCalculations.getTargetShootingState().targetShootingVelocityRotationsPerSecond(),
                targetLeftVelocityRotationsPerSecond = targetRightVelocityRotationsPerSecond * ShooterConstants.LEFT_MOTOR_TO_RIGHT_MOTOR_RATIO;
        setTargetVelocity(targetRightVelocityRotationsPerSecond, targetLeftVelocityRotationsPerSecond);
    }

    void setTargetVelocity(double targetRightVelocityRotationsPerSecond, double targetLeftVelocityRotationsPerSecond) {
        setTargetRightVelocity(targetRightVelocityRotationsPerSecond);
        setTargetLeftVelocity(targetLeftVelocityRotationsPerSecond);
    }

    private void setTargetRightVelocity(double targetVelocityRotationsPerSecond) {
        ShooterConstants.RIGHT_MECHANISM.setTargetVelocity(targetVelocityRotationsPerSecond);
        rightMotor.setControl(velocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    private void setTargetLeftVelocity(double targetVelocityRotationsPerSecond) {
        ShooterConstants.LEFT_MECHANISM.setTargetVelocity(targetVelocityRotationsPerSecond);
        leftMotor.setControl(velocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    private void updateMechanism() {
        ShooterConstants.RIGHT_MECHANISM.update(rightMotor.getSignal(TalonFXSignal.VELOCITY));
        ShooterConstants.LEFT_MECHANISM.update(leftMotor.getSignal(TalonFXSignal.VELOCITY));
    }
}
