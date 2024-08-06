package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
    private final TorqueCurrentFOC velocityRequest = new TorqueCurrentFOC(0);
    private double
            targetRightVelocityRotationsPerSecond = 0,
            targetLeftVelocityRotationsPerSecond = 0;

    public Shooter() {
        setName("Shooter");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("RightShooter")
                .linearPosition(Units.Meters.of(rightMotor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(rightMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(rightMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
        log.motor("LeftShooter")
                .linearPosition(Units.Meters.of(leftMotor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(leftMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(leftMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
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

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        leftMotor.setControl(velocityRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return ShooterConstants.SYSID_CONFIG;
    }

    void reachTargetShootingVelocityFromShootingCalculations() {
        targetRightVelocityRotationsPerSecond = shootingCalculations.getTargetShootingState().targetShootingVelocityRotationsPerSecond();
        targetLeftVelocityRotationsPerSecond = targetRightVelocityRotationsPerSecond * ShooterConstants.LEFT_MOTOR_TO_RIGHT_MOTOR_RATIO;
        setTargetVelocity(targetRightVelocityRotationsPerSecond, targetLeftVelocityRotationsPerSecond);
    }

    void setTargetVelocity(double targetRightVelocityRotationsPerSecond, double targetLeftVelocityRotationsPerSecond) {
        setTargetRightVelocity(targetRightVelocityRotationsPerSecond);
        setTargetLeftVelocity(targetLeftVelocityRotationsPerSecond);
    }

    void setTargetRightVelocity(double targetVelocityRotationsPerSecond) {
        rightMotor.setControl(velocityRequest.withOutput(targetVelocityRotationsPerSecond));
    }

    void setTargetLeftVelocity(double targetVelocityRotationsPerSecond) {
        leftMotor.setControl(velocityRequest.withOutput(targetVelocityRotationsPerSecond));
    }

    private void updateMechanism() {
        ShooterConstants.RIGHT_MECHANISM.update(rightMotor.getSignal(TalonFXSignal.VELOCITY), targetRightVelocityRotationsPerSecond);
        ShooterConstants.LEFT_MECHANISM.update(leftMotor.getSignal(TalonFXSignal.VELOCITY), targetLeftVelocityRotationsPerSecond);
    }
}
