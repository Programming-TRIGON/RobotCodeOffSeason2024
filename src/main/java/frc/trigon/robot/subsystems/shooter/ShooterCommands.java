package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;

public class ShooterCommands {
    public static Command getReachTargetShootingVelocityFromShootingCalculations() {
        return new RunCommand(
                RobotContainer.SHOOTER::reachTargetShootingVelocityFromShootingCalculations,
                RobotContainer.SHOOTER
        );
    }

    public static Command getSetTargetVelocity(double rightMotorTargetVelocityRotationsPerSecond, double leftMotorTargetVelocityRotationsPerSecond) {
        return new RunCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(rightMotorTargetVelocityRotationsPerSecond, leftMotorTargetVelocityRotationsPerSecond),
                RobotContainer.SHOOTER
        );
    }

    public static Command getSetRightTargetVelocity(double targetVelocityRotationsPerSecond) {
        return new RunCommand(
                () -> RobotContainer.SHOOTER.setRightTargetVelocity(targetVelocityRotationsPerSecond),
                RobotContainer.SHOOTER
        );
    }

    public static Command getSetLeftTargetVelocity(double targetVelocityRotationsPerSecond) {
        return new RunCommand(
                () -> RobotContainer.SHOOTER.setLeftTargetVelocity(targetVelocityRotationsPerSecond),
                RobotContainer.SHOOTER
        );
    }
}
