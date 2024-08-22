package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
    private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
    private double
            targetRightVelocityRotationsPerSecond = 0,
            targetLeftVelocityRotationsPerSecond = 0;

    public Shooter() {
        setName("Shooter");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("RightShooter")
                .angularPosition(Units.Rotations.of(rightMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(rightMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(rightMotor.getSignal(TalonFXSignal.TORQUE_CURRENT)));
        log.motor("LeftShooter")
                .angularPosition(Units.Rotations.of(leftMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(leftMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(leftMotor.getSignal(TalonFXSignal.TORQUE_CURRENT)));
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
        rightMotor.setControl(torqueRequest.withOutput(voltageMeasure.in(Units.Volts)));
        leftMotor.setControl(torqueRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ShooterConstants.SYSID_CONFIG;
    }

    public boolean bothMotorsAtTargetVelocity() {
        return atTargetRightVelocity() && atTargetLeftVelocity();
    }

    public boolean atTargetRightVelocity() {
        return atVelocity(targetRightVelocityRotationsPerSecond);
    }

    public boolean atTargetLeftVelocity() {
        return atVelocity(targetLeftVelocityRotationsPerSecond);
    }

    public boolean atVelocity(double targetVelocity) {
        return Math.abs(leftMotor.getSignal(TalonFXSignal.VELOCITY) - targetVelocity) < ShooterConstants.VELOCITY_TOLERANCE;
    }

    void reachTargetShootingVelocityFromShootingCalculations() {
        final double
                targetRightVelocityRotationsPerSecond = shootingCalculations.getTargetShootingState().targetShootingVelocityRotationsPerSecond(),
                targetLeftVelocityRotationsPerSecond = targetRightVelocityRotationsPerSecond * ShooterConstants.RIGHT_MOTOR_TO_LEFT_MOTOR_RATIO;
        setTargetVelocity(targetRightVelocityRotationsPerSecond, targetLeftVelocityRotationsPerSecond);
    }

    void setTargetVelocity(double targetRightVelocityRotationsPerSecond, double targetLeftVelocityRotationsPerSecond) {
        setTargetRightVelocity(targetRightVelocityRotationsPerSecond);
        setTargetLeftVelocity(targetLeftVelocityRotationsPerSecond);
    }

    private void setTargetRightVelocity(double targetVelocityRotationsPerSecond) {
        targetRightVelocityRotationsPerSecond = targetVelocityRotationsPerSecond;
        ShooterConstants.RIGHT_MECHANISM.setTargetVelocity(targetVelocityRotationsPerSecond);
        rightMotor.setControl(velocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    private void setTargetLeftVelocity(double targetVelocityRotationsPerSecond) {
        targetLeftVelocityRotationsPerSecond = targetVelocityRotationsPerSecond;
        ShooterConstants.LEFT_MECHANISM.setTargetVelocity(targetVelocityRotationsPerSecond);
        leftMotor.setControl(velocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    private void updateMechanism() {
        ShooterConstants.RIGHT_MECHANISM.update(rightMotor.getSignal(TalonFXSignal.VELOCITY));
        ShooterConstants.LEFT_MECHANISM.update(leftMotor.getSignal(TalonFXSignal.VELOCITY));
    }
}
