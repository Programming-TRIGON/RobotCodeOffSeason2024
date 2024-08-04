package frc.trigon.robot.subsystems.climber;

import frc.trigon.robot.hardware.phoenix6.talonfx.TalonFXMotor;
import frc.trigon.robot.subsystems.MotorSubsystem;

public class Climber extends MotorSubsystem {
    private final TalonFXMotor
            masterMotor = ClimberConstants.MASTER_MOTOR,
            followerMotor = ClimberConstants.FOLLOWER_MOTOR;

    public Climber() {
        setName("Climber");
    }

    @Override
    public void periodic() {
        masterMotor.update();
        followerMotor.update();
        updateMechanism();
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

    private void updateMechanism() {
        ClimberConstants.MECHANISM.update();
    }
}
