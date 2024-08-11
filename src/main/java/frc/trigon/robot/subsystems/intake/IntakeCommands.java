package frc.trigon.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.*;
import frc.trigon.robot.RobotContainer;

public class IntakeCommands {
    public static Command getSetTargetStateAndWaitCommand(IntakeConstants.IntakeState intakeState) {
        return new SequentialCommandGroup(
                getSetTargetStateCommand(intakeState),
                getWaitCommand().andThen(getStopIntakeCommand())
        );
    }

    public static Command getSetTargetStateCommand(IntakeConstants.IntakeState targetState) {
        if (targetState == IntakeConstants.IntakeState.COLLECTING) {
            return new FunctionalCommand(
                    () -> RobotContainer.INTAKE.setTargetState(targetState),
                    () -> RobotContainer.INTAKE.setBrake(true),
                    (interrupted) -> {
                        RobotContainer.INTAKE.stop();
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
        RobotContainer.INTAKE.setBrake(true);
        return new WaitCommand(IntakeConstants.WAIT_COMMAND_SECONDS);
    }
}