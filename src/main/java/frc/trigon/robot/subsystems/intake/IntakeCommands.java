package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.NetworkTablesCommand;

public class IntakeCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                IntakeCommands::getSetTargetVoltageCommand,
                false,
                "Debugging/TargetDebuggingIntakeVoltage"
        );
    }
    
    public static Command getCollectionCommand() {
        return new SequentialCommandGroup(
                getSetTargetStateCommand(IntakeConstants.IntakeState.COLLECTING),
                getWaitCommand().andThen(getStopIntakeCommand())
        );
    }

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        if (targetState == IntakeConstants.IntakeState.COLLECTING) {
            return new FunctionalCommand(
                    () -> RobotContainer.INTAKE.setTargetState(targetState),
                    () -> {
                    },
                    (interrupted) -> {
                        RobotContainer.INTAKE.sendStaticBrakeRequest();
                        RobotContainer.INTAKE.indicateCollection();
                    },
                    RobotContainer.INTAKE::hasNote,
                    RobotContainer.INTAKE
            );
        }
        return getSetTargetVoltageCommand(targetState.voltage);
    }

    public static Command getSetTargetVoltageCommand(double targetVoltage) {
        return new StartEndCommand(
                () -> RobotContainer.INTAKE.setTargetVoltage(targetVoltage),
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    public static Command getStopIntakeCommand() {
        return new InstantCommand(
                RobotContainer.INTAKE::stop,
                RobotContainer.INTAKE
        );
    }

    private static Command getWaitCommand() {
        return new WaitCommand(IntakeConstants.COLLECTION_CONFERMATION_SECONDS);
    }
}