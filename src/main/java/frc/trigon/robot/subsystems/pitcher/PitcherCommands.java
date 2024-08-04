package frc.trigon.robot.subsystems.pitcher;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.trigon.robot.RobotContainer;

public class PitcherCommands {
    public static Command getReachTargetPitchFromShootingCalculationsCommand() {
        return new RunCommand(
                RobotContainer.PITCHER::reachTargetPitchFromShootingCalculations,
                RobotContainer.PITCHER
        );
    }

    public static Command getSetPositionCommand(Rotation2d targetPitch) {
        return new RunCommand(
                () -> RobotContainer.PITCHER.setPosition(targetPitch),
                RobotContainer.PITCHER
        );
    }
}
