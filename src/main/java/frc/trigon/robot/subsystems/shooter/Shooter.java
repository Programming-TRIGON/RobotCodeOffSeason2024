package frc.trigon.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.Conversions;
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

    public void updateLog(SysIdRoutineLog log) {
        log.motor("RightShooter")
                .linearPosition(Units.Meters.of(toMeters(rightMotor.getSignal(TalonFXSignal.POSITION))))
                .linearVelocity(Units.MetersPerSecond.of(rightMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(rightMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
        log.motor("LeftShooter")
                .linearPosition(Units.Meters.of(toMeters(leftMotor.getSignal(TalonFXSignal.POSITION))))
                .linearVelocity(Units.MetersPerSecond.of(toMeters(leftMotor.getSignal(TalonFXSignal.VELOCITY))))
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

    public SysIdRoutine.Config getSysIdConfig() {
        return ShooterConstants.SYSID_CONFIG;
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

    private static double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ShooterConstants.WHEEL_DIAMETER_METERS);
    }
}
