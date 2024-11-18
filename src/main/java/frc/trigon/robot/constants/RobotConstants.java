package frc.trigon.robot.constants;

import frc.trigon.robot.Robot;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.FilesHandler;

public class RobotConstants {
    private static final RobotHardwareStats.ReplayType REPLAY_TYPE = RobotHardwareStats.ReplayType.NONE;
    private static final double PERIODIC_TIME_SECONDS = 0.02;
    public static final String CANIVORE_NAME = "CANivore";
    public static final String LOGGING_PATH = Robot.IS_REAL ? "/media/sda1/akitlogs/" : FilesHandler.DEPLOY_PATH + "logs/";

    public static void init() {
        RobotHardwareStats.setCurrentRobotStats(Robot.IS_REAL, REPLAY_TYPE);
        RobotHardwareStats.setPeriodicTimeSeconds(PERIODIC_TIME_SECONDS);
    }
}