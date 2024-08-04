package frc.trigon.robot.subsystems.pitcher;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXSignal;
import frc.trigon.robot.subsystems.MotorSubsystem;
import frc.trigon.robot.utilities.ShootingCalculations;

public class Pitcher extends MotorSubsystem {
    private final ShootingCalculations shootingCalculations = ShootingCalculations.getInstance();
    private final TalonFXMotor motor = PitcherConstants.MOTOR;
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(PitcherConstants.FOC_ENABLED);
    private Rotation2d targetPosition;

    public Pitcher() {
        setName("Pitcher");
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
        PitcherConstants.MECHANISM.update(Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.POSITION)), Rotation2d.fromRotations(motor.getSignal(TalonFXSignal.CLOSED_LOOP_REFERENCE)));
    }
}
