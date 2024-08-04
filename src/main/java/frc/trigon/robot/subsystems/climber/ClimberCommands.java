package frc.trigon.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class ClimberCommands {

    public static Command getSetTargetPositionCommand(double targetPositionMeters) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetPosition(targetPositionMeters),
                RobotContainer.CLIMBER::stop,
                RobotContainer.CLIMBER
        );
    }

    public static Command getSetTargetStateCommand(ClimberConstants.ClimberState targetState) {
        return new StartEndCommand(
                () -> RobotContainer.CLIMBER.setTargetState(targetState),
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
