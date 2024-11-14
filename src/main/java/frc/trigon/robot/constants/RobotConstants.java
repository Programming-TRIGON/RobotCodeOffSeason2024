package frc.trigon.robot.constants;

import frc.trigon.robot.Robot;
import org.trigon.hardware.RobotHardwareStats;
import org.trigon.utilities.FilesHandler;

public class RobotConstants {
    private static final boolean
            IS_SIMULATION = true,
            IS_REPLAY = false;
    private static final RobotHardwareStats.ReplayType REPLAY_TYPE = IS_SIMULATION ? RobotHardwareStats.ReplayType.SIMULATION_REPLAY : RobotHardwareStats.ReplayType.REAL_REPLAY;
    private static final double PERIODIC_TIME_SECONDS = 0.02;
    public static final String CANIVORE_NAME = "CANivore";
    public static final String LOGGING_PATH = IS_SIMULATION && !Robot.IS_REAL ? FilesHandler.DEPLOY_PATH + "logs/" : "/media/sda1/akitlogs/";

    public static void init() {
        RobotHardwareStats.setCurrentRobotStats(Robot.IS_REAL, IS_REPLAY ? REPLAY_TYPE : RobotHardwareStats.ReplayType.NONE);
        RobotHardwareStats.setPeriodicTimeSeconds(PERIODIC_TIME_SECONDS);
    }
}