package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public class RobotPoseSource {
    protected final String name;
    private final RobotPoseSourceInputsAutoLogged inputs = new RobotPoseSourceInputsAutoLogged();
    private final Transform3d robotCenterToCamera;
    private final RobotPoseSourceIO robotPoseSourceIO;
    private double lastUpdatedTimestamp;
    private Pose2d cachedPose = null;

    public RobotPoseSource(RobotPoseSourceConstants.RobotPoseSourceType robotPoseSourceType, String name, Transform3d robotCenterToCamera) {
        this.name = name;
        if (robotPoseSourceType != RobotPoseSourceConstants.RobotPoseSourceType.PHOTON_CAMERA)
            this.robotCenterToCamera = robotCenterToCamera;
        else
            this.robotCenterToCamera = new Transform3d();

        if (Robot.IS_REAL)
            robotPoseSourceIO = robotPoseSourceType.createIOFunction.apply(name, robotCenterToCamera);
        else
            robotPoseSourceIO = new RobotPoseSourceIO();
    }

    public void update() {
        robotPoseSourceIO.updateInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);
        cachedPose = getUnCachedRobotPose();
        if (!inputs.hasResult || cachedPose == null)
            Logger.recordOutput("Poses/Robot/" + name + "Pose", RobotPoseSourceConstants.EMPTY_POSE_LIST);
        else
            Logger.recordOutput("Poses/Robot/" + name + "Pose", cachedPose);
    }

    public int getVisibleTags() {
        return inputs.visibleTagIDs.length;
    }

    public double getAverageDistanceFromTags() {
//        return inputs.averageDistanceFromTags;
        return 0;
    }

    public boolean hasNewResult() {
        return inputs.hasResult && isNewTimestamp();
    }

    public Pose2d getRobotPose() {
        return cachedPose;
    }

    public String getName() {
        return name;
    }

    public double getLastResultTimestamp() {
        return inputs.lastResultTimestamp;
    }

    private Pose2d getUnCachedRobotPose() {
        final Pose3d cameraPose = inputs.solvePNPPose;
        if (cameraPose == null)
            return null;

        return cameraPose.transformBy(robotCenterToCamera.inverse()).toPose2d();
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }
}
