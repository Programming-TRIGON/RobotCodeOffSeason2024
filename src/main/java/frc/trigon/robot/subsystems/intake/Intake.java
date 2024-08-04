package frc.trigon.robot.subsystems.intake;

import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Intake extends MotorSubsystem {
    private final TalonFXMotor
            masterMotor = IntakeConstants.MASTER_MOTOR,
            followerMotor = IntakeConstants.FOLLOWER_MOTOR;

    public Intake() {

    }

    @Override
    public void setBrake(boolean brake) {

    }

    @Override
    public void stop() {

    }
}
