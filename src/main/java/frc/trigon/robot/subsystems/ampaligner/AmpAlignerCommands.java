package frc.trigon.robot.subsystems.ampaligner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.trigon.robot.RobotContainer;

public class AmpAlignerCommands {
    public static Command getOpenAmpAlignerCommand() {
        return new StartEndCommand(
                RobotContainer.AMP_ALIGNER::openAmpAligner,
                RobotContainer.AMP_ALIGNER::stop,
                RobotContainer.AMP_ALIGNER
        );
    }

    public static Command getCloseAmpAlignerCommand() {
        return new StartEndCommand(
                RobotContainer.AMP_ALIGNER::closeAmpAligner,
                RobotContainer.AMP_ALIGNER::stop,
                RobotContainer.AMP_ALIGNER
        );
    }
}
