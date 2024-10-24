package frc.trigon.robot.subsystems.ampaligner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;
import org.trigon.commands.ExecuteEndCommand;
import org.trigon.commands.NetworkTablesCommand;

public class AmpAlignerCommands {
    public static Command getDebuggingCommand() {
        return new NetworkTablesCommand(
                (targetAngleDegrees) -> AmpAlignerCommands.getSetTargetAngleCommand(Rotation2d.fromDegrees(targetAngleDegrees)),
                true,
                "Debugging/TargetDebuggingAmpAlignerDegrees"
        );
    }

    public static Command getSetTargetAngleCommand(Rotation2d targetAngle) {
        return new ExecuteEndCommand(
                () -> RobotContainer.AMP_ALIGNER.setTargetAngle(targetAngle),
                RobotContainer.AMP_ALIGNER::stop,
                RobotContainer.AMP_ALIGNER
        );
    }

    public static Command getSetTargetStateCommand(AmpAlignerConstants.AmpAlignerState targetState) {
        return new ExecuteEndCommand(
                () -> RobotContainer.AMP_ALIGNER.setTargetState(targetState),
                RobotContainer.AMP_ALIGNER::stop,
                RobotContainer.AMP_ALIGNER
        );
    }

    public static Command getSetTargetVoltageCommand(double voltage) {
        return new StartEndCommand(
                () -> RobotContainer.AMP_ALIGNER.drive(Units.Volts.of(voltage)),
                RobotContainer.AMP_ALIGNER::stop
        );
    }

    public static Command getSetPositionCommand(Rotation2d position) {
        return new InstantCommand(
                () -> RobotContainer.AMP_ALIGNER.setPosition(position)
        );
    }
}
