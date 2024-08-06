package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class ClimberCommands {
    public static Command getSetTargetPositionCommand(double targetRightPositionMeters, double targetLeftPositionMeters) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetPosition(targetRightPositionMeters, targetLeftPositionMeters),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetRightState, ClimberConstants.ClimberState targetLeftState) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetRightState, targetLeftState),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetRightMotorTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return getSetRightMotorTargetPositionCommand(targetState.positionMeters);
    }

    public static Command getSetLeftMotorTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return getSetLeftMotorTargetPositionCommand(targetState.positionMeters);
    }

    public static Command getSetRightMotorTargetPositionCommand(double targetPositionMeters) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setRightMotorTargetPosition(targetPositionMeters),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetLeftMotorTargetPositionCommand(double targetPositionMeters) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setLeftMotorTargetPosition(targetPositionMeters),
                RobotContainer.CLIMBER::stop,
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
