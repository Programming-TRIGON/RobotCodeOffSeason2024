package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.NetworkTablesCommand;

public class ClimberCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (positionRotations, affectedByRobotWeight) -> ClimberCommands.getSetTargetPositionCommand(positionRotations, positionRotations, affectedByRobotWeight == 1),
                true,
                "Debugging/TargetDebuggingClimberPositionRotations",
                "Debugging/TargetDebuggingClimberPositionAffectedByRobotWeight"
        );
    }

    public static Command getSetTargetPositionCommand(double targetRightPositionRotations, double targetLeftPositionRotations, boolean affectedByRobotWeight) {
        return new ExecuteEndCommand(
                () -> RobotContainer.CLIMBER.setTargetPosition(targetRightPositionRotations, targetLeftPositionRotations, affectedByRobotWeight),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new ExecuteEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetState),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return getSetTargetVoltageCommand(targetVoltage, targetVoltage);
    }

    public static Command getSetTargetVoltageCommand(double rightTargetVoltage, double leftTargetVoltage) {
        return new StartEndCommand(
                () -> {
                    RobotContainer.CLIMBER.driveRightMotor(Units.Volt.of(rightTargetVoltage));
                    RobotContainer.CLIMBER.driveLeftMotor(Units.Volt.of(leftTargetVoltage));
                },
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getStopCommand() {
        return new InstantCommand(
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }
}
