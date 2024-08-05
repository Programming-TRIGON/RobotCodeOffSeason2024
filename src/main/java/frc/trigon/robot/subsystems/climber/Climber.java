package frc.trigon.robot.subsystems.climber;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.phoenix6.talonfx.TalonFXMotor;
import org.trigon.hardware.phoenix6.talonfx.TalonFXSignal;
import org.trigon.utilities.Conversions;
import org.trigon.utilities.MotorSubsystem;

public class Climber extends MotorSubsystem {
    private final TalonFXMotor
            masterMotor = ClimberConstants.MASTER_MOTOR,
            followerMotor = ClimberConstants.FOLLOWER_MOTOR;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(ClimberConstants.ENABLE_FOC);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(ClimberConstants.ENABLE_FOC);

    public Climber() {
        setName("Climber");
    }

    @Override
    public void periodic() {
        masterMotor.update();
        followerMotor.update();
        updateMechanism();
    }

    public void drive(Measure<Voltage> voltageMeasure) {
        masterMotor.setControl(voltageRequest.withOutput(voltageMeasure.in(Units.Volts)));
    }

    public void updateLog(SysIdRoutineLog log) {
        log.motor("Climber")
                .linearPosition(Units.Meters.of(masterMotor.getSignal(TalonFXSignal.POSITION)))
                .linearVelocity(Units.MetersPerSecond.of(masterMotor.getSignal(TalonFXSignal.VELOCITY)))
                .voltage(Units.Volts.of(masterMotor.getSignal(TalonFXSignal.MOTOR_VOLTAGE)));
    }

    @Override
    public void setBrake(boolean brake) {
        masterMotor.setBrake(brake);
        followerMotor.setBrake(brake);
    }

    public SysIdRoutine.Config getSysIdConfig() {
        return ClimberConstants.SYSID_CONFIG;
    }

    @Override
    public void stop() {
        masterMotor.stopMotor();
    }

    void setTargetState(ClimberConstants.ClimberState targetState) {
        setTargetPosition(targetState.positionMeters);
    }

    void setTargetPosition(double targetPositionMeters) {
        masterMotor.setControl(motionMagicRequest.withPosition(targetPositionMeters));
    }

    private void updateMechanism() {
        ClimberConstants.MECHANISM.update(
                toMeters(masterMotor.getSignal(TalonFXSignal.POSITION)),
                toMeters(masterMotor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)));
        Logger.recordOutput("Poses/Components/ClimberPose", getClimberPose());
    }

    private Pose3d getClimberPose() {
        final Transform3d climberTransform = new Transform3d(
                new Translation3d(0, 0, toMeters(masterMotor.getSignal(TalonFXSignal.POSITION))),
                new Rotation3d()
        );
        return ClimberConstants.CLIMBER_ORIGIN_POINT.transformBy(climberTransform);
    }

    private double toMeters(double rotations) {
        return Conversions.rotationsToDistance(rotations, ClimberConstants.DRUM_DIAMETER_METERS);
    }
}
