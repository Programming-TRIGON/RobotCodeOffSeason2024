package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class IntakeCommands {

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        return getSetTargetVoltageSequentialCommand(targetState.voltage);
    }

    public static Command getSetTargetVoltageSequentialCommand(double collectorVoltage) {
        return new SequentialCommandGroup(
                getSetTargetVoltageCommand(collectorVoltage).until(RobotContainer.INTAKE::hasNote),
                getStopIntakeCommand()
        );
    }

    private static Command getSetTargetVoltageCommand(double collectorVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetVoltage(collectorVoltage),
                () -> {
                },
                RobotContainer.INTAKE
        );
    }

    public static Command getStopIntakeCommand() {
        return new StartEndCommand(
                RobotContainer.INTAKE::stop,
                () -> {
                },
                RobotContainer.INTAKE
        );
    }
}
