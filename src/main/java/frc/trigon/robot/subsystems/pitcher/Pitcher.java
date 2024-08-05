package frc.trigon.robot.subsystems.pitcher;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.MotorSubsystem;

public class Pitcher extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor motor = PitcherConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(PitcherConstants.FOC_ENABLED);
    private Rotation2d targetPosition;

    public Pitcher() {
        setName("Pitcher");
    }

    public void updateLog(SysIdRoutineLog log) {
        log.motor("Pitcher")
                .linearPosition(Units.Meters.of(toMeters(motor.getSignal(TalonFXSignal.POSITION))))
                .linearVelocity(Units.MetersPerSecond.of(toMeters(motor.getSignal(TalonFXSignal.VELOCITY))))
                .voltage(Units.Volts.of(motor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        motor.update();
        updateMechanism();
    }

    public SysIdRoutine.Config getSysIdConfig() {
        return PitcherConstants.SYSID_CONFIG;
    }

    public Rotation2d getTargetPitch() {
        return targetPosition;
    }

    void reachTargetPitchFromShootingCalculations() {
        targetPosition = shootingCalculations.getTargetShootingState().targetPitch();
        setPosition(targetPosition);
    }

    void setPosition(Rotation2d targetPosition) {
        this.targetPosition = targetPosition;
        motor.setControl(positionRequest.withPosition(targetPosition.getRotations()));
    }

    private void updateMechanism() {
        PitcherConstants.MECHANISM.update(
                Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION)),
                Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE))
        );
    }

    private static double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, PitcherConstants.WHEEL_DIAMETER_METERS);
    }
}
