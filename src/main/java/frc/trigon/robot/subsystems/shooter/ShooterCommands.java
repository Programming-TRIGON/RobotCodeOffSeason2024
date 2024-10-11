package frc.trigon.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.NetworkTablesCommand;

public class ShooterCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                ShooterCommands::getSetTargetVelocityCommand,
                false,
                "Debugging/TargetRightMotorDebuggingShootingVelocity",
                "Debugging/TargetLeftMotorDebuggingShootingVelocity"
        );
    }

    public static Command getReachTargetShootingVelocityFromShootingCalculationsCommand() {
        return new ExecuteEndCommand(
                RobotContainer.SHOOTER::reachTargetShootingVelocityFromShootingCalculations,
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getSetTargetVelocityCommand(double targetVelocityBothMotors) {
        return new StartEndCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(targetVelocityBothMotors, targetVelocityBothMotors),
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getSetTargetVelocityCommand(double targetRightVelocityRotationsPerSecond, double targetLeftVelocityRotationsPerSecond) {
        return new StartEndCommand(
                () -> RobotContainer.SHOOTER.setTargetVelocity(targetRightVelocityRotationsPerSecond, targetLeftVelocityRotationsPerSecond),
                RobotContainer.SHOOTER::stop,
                RobotContainer.SHOOTER
        );
    }

    public static Command getSendStaticBreakRequestCommand() {
        return new StartEndCommand(
                RobotContainer.SHOOTER::sendStaticBrakeRequest,
                () -> {
                },
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
