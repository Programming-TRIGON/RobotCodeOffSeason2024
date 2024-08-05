package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class ClimberCommands {
    public static Command getSetTargetPositionCommand(double targetPositionMeters, double leftTargetMeters) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetPosition(targetPositionMeters, leftTargetMeters),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState rightTargetState, ClimberConstants.ClimberState leftTargetState) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(rightTargetState, leftTargetState),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetRightMotorTargetStateCommand(ClimberConstants.ClimberState rightTargetState) {
        return getSetRightMotorTargetPositionCommand(rightTargetState.positionMeters);
    }

    public static Command getSetLeftMotorTargetStateCommand(ClimberConstants.ClimberState leftTargetState) {
        return getSetLeftMotorTargetPositionCommand(leftTargetState.positionMeters);
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
