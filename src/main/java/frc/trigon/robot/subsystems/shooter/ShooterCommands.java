package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;

public class ShooterCommands {
    private static final Shooter SHOOTER = RobotContainer.SHOOTER;

    public static Command getReachTargetShootingVelocityFromShootingCalculations() {
        return new RunCommand(
                SHOOTER::reachTargetShootingVelocityFromShootingCalculations,
                SHOOTER
        );
    }

    public static Command getSetTargetVelocity(double rightMotorTargetVelocityRotationsPerSecond, double leftMotorTargetVelocityRotationsPerSecond) {
        return new RunCommand(
                () -> SHOOTER.setTargetVelocity(rightMotorTargetVelocityRotationsPerSecond, leftMotorTargetVelocityRotationsPerSecond),
                SHOOTER
        );
    }

    public static Command getSetRightTargetVelocity(double targetVelocityRotationsPerSecond) {
        return new RunCommand(
                () -> SHOOTER.setRightTargetVelocity(targetVelocityRotationsPerSecond),
                SHOOTER
        );
    }

    public static Command getSetLeftTargetVelocity(double targetVelocityRotationsPerSecond) {
        return new RunCommand(
                () -> SHOOTER.setLeftTargetVelocity(targetVelocityRotationsPerSecond),
                SHOOTER
        );
    }
}
