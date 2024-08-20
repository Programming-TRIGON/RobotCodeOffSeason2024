package frc.trigon.robot.subsystems.pitcher;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.ShootingCalculations;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;

public class Pitcher extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor
            masterMotor = PitcherConstants.MASTER_MOTOR,
            followerMotor = PitcherConstants.FOLLOWER_MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(PitcherConstants.FOC_ENABLED);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(PitcherConstants.FOC_ENABLED);
    private Rotation2d targetPitch = PitcherConstants.DEFAULT_PITCH;

    public Pitcher() {
        setName("Pitcher");
    }

    @Override
    public void updateLog(SysIdRoutineLog log) {
        log.motor("Pitcher")
                .angularPosition(Units.Rotations.of(masterMotor.getSignal(TalonFXSignal.POSITION)))
                .angularVelocity(Units.RotationsPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void setBrake(boolean brake) {
        masterMotor.setBrake(brake);
        followerMotor.setBrake(brake);
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    @Override
    public void periodic() {
        masterMotor.update();
        updateMechanism();
    }

    @Override
    public void drive(Measure<Voltage> voltageMeasure) {
        masterMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    @Override
    public SysIdRoutine.Config getSysIdConfig() {
        return PitcherConstants.SYSID_CONFIG;
    }

    public Rotation2d getCurrentPitch() {
        return Rotation2d.fromRotations(masterMotor.getSignal(TalonFXSignal.POSITION));
    }

    public Rotation2d getTargetPitch() {
        return targetPitch;
    }

    public boolean atTargetPitch() {
        return Math.abs(masterMotor.getSignal(TalonFXSignal.POSITION) - targetPitch.getRotations()) < PitcherConstants.PITCH_TOLERANCE.getRotations();
    }

    void reachTargetPitchFromShootingCalculations() {
        targetPitch = shootingCalculations.getTargetShootingState().targetPitch();
        setTargetPitch(targetPitch);
    }

    void setTargetPitch(Rotation2d targetPitch) {
        this.targetPitch = targetPitch;
        masterMotor.setControl(positionRequest.withPosition(targetPitch.getRotations()));
    }

    private void updateMechanism() {
        PitcherConstants.PITCHER_AND_AMP_ALIGNER_MECHANISM.updateFirstJoint(
                getCurrentPitch(),
                Rotation2d.fromRotations(targetPitch.getRotations())
        );
    }
}
