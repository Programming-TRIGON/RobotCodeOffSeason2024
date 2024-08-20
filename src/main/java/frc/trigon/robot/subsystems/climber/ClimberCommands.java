package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

public class ClimberCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (positionMeters, affectedByRobotWeight) -> ClimberCommands.getSetTargetPositionCommand(positionMeters, positionMeters, affectedByRobotWeight == 1),
                true,
                "Debugging/TargetDebuggingClimberPositionMeters",
                "Debugging/TargetDebuggingClimberPositionAffectedByRobotWeight"
        );
    }

    public static Command getSetTargetPositionCommand(double targetRightPositionMeters, double targetLeftPositionMeters, boolean affectedByRobotWeight) {
        return new FunctionalCommand(
                () -> {
                },
                () -> RobotContainer.CLIMBER.setTargetPosition(targetRightPositionMeters, targetLeftPositionMeters, affectedByRobotWeight),
                (interrupted) -> RobotContainer.CLIMBER.stop(),
                () -> false,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new FunctionalCommand(
                () -> {
                },
                () -> RobotContainer.CLIMBER.setTargetState(targetState),
                (interrupted) -> RobotContainer.CLIMBER.stop(),
                () -> false,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.drive(Units.Volt.of(targetVoltage)),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getStopCommand() {
        return new StartEndCommand(
                RobotContainer.CLIMBER::stop,
                () -> {
                },
                RobotContainer.CLIMBER
        );
    }
}
