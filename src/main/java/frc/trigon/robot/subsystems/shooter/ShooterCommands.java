package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.NetworkTablesCommand;

public class ShooterCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetVelocity) -> ShooterCommands.getSetTargetVelocity(targetVelocity, targetVelocity),
                false,
                "Debugging/TargetDebuggingShootingVelocity"
        );
    }

    public static Command getReachTargetShootingVelocityFromShootingCalculations() {
        return new ExecuteEndCommand(
                RobotContainer.SHOOTER::reachTargetShootingVelocityFromShootingCalculations,
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getSetTargetVelocity(double targetRightVelocityRotationsPerSecond, double targetLeftVelocityRotationsPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(targetRightVelocityRotationsPerSecond, targetLeftVelocityRotationsPerSecond),
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.SHOOTER::stop,
                () -> {
                },
                RobotContainer.SHOOTER
        );
    }
}