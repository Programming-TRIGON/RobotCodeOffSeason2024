package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.geometry.*;
import frc.trigon.robot.Robot;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

/**
 * A pose source is a class that provides the robot's pose, from a camera.
 */
public class RobotPoseSource {
    protected final String name;
    private final RobotPoseSourceInputsAutoLogged inputs = new RobotPoseSourceInputsAutoLogged();
    private final Transform3d robotCenterToCamera;
    private final RobotPoseSourceIO robotPoseSourceIO;
    private double lastUpdatedTimestamp;
    private Pose2d robotPose = null;

    public RobotPoseSource(RobotPoseSourceConstants.RobotPoseSourceType robotPoseSourceType, String name, Transform3d robotCenterToCamera) {
        this.name = name;
        this.robotCenterToCamera = robotCenterToCamera;

        if (Robot.IS_REAL)
            robotPoseSourceIO = robotPoseSourceType.createIOFunction.apply(name);
        else
            robotPoseSourceIO = new RobotPoseSourceIO();
    }

    public void update() {
        robotPoseSourceIO.updateInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);

        robotPose = getBestRobotPose(inputs.averageDistanceFromAllTags);

        if (!inputs.hasResult || inputs.averageDistanceFromAllTags == 0 || robotPose == null)
            Logger.recordOutput("Poses/Robot/" + name + "Pose", RobotPoseSourceConstants.EMPTY_POSE_LIST);
        else
            Logger.recordOutput("Poses/Robot/" + name + "Pose", robotPose);
    }

    public boolean hasNewResult() {
        return (inputs.hasResult && inputs.averageDistanceFromAllTags != 0) && isNewTimestamp();
    }

    /**
     * @return the current estimated robot position
     */
    public Pose2d getCurrentRobotPose() {
        return robotPose;
    }

    public String getName() {
        return name;
    }

    public double getAverageDistanceFromTags() {
        return inputs.averageDistanceFromAllTags;
    }

    public int getNumberOfVisibleTags() {
        return inputs.visibleTagIDs.length;
    }

    public double getLastResultTimestamp() {
        return inputs.lastResultTimestamp;
    }

    /**
     * Gets the best possible robot Pose from the camera's data
     *
     * @param averageDistanceFromAllTags the average of the distance from all visible tags
     * @return the result
     */
    private Pose2d getBestRobotPose(double averageDistanceFromAllTags) {
        if (averageDistanceFromAllTags < RobotPoseSourceConstants.MAXIMUM_DISTANCE_FROM_TAG_FOR_PNP_METERS)
            return getRobotPoseFromCameraPose(inputs.solvePNPPose);
        return getAssumedRobotHeadingPose().toPose2d();
    }

    private Pose3d getAssumedRobotHeadingPose() {
        final Rotation2d robotHeading = RobotContainer.SWERVE.getHeading();
        final Translation2d robotFieldRelativePositionTranslation = getRobotFieldRelativePosition(robotHeading);
        return new Pose3d(new Pose2d(robotFieldRelativePositionTranslation, robotHeading));
    }

    private Translation2d getRobotFieldRelativePosition(Rotation2d robotHeading) {
        final Pose3d tagPose = FieldConstants.TAG_ID_TO_POSE.get(inputs.visibleTagIDs[0]);
        final Translation2d tagToCamera = getTagToCamera(tagPose);
        final Translation2d robotToTag = tagToCamera.plus(robotCenterToCamera.getTranslation().toTranslation2d());
        final Translation2d fieldRelativeTagPose = robotToTag.rotateBy(robotHeading);
        return tagPose.getTranslation().toTranslation2d().plus(fieldRelativeTagPose);
    }

    private Translation2d getTagToCamera(Pose3d tagPose) {
        final double cameraToTagDistanceMeters = -PhotonUtils.calculateDistanceToTargetMeters(
                robotCenterToCamera.getZ(), tagPose.getZ(), robotCenterToCamera.getRotation().getY(), inputs.bestTargetRelativePitch
        );
        final double cameraToTagXDistance = Math.sin(inputs.bestTargetRelativeYaw + robotCenterToCamera.getRotation().getZ()) * cameraToTagDistanceMeters;
        final double cameraToTagYDistance = -Math.cos(inputs.bestTargetRelativeYaw + robotCenterToCamera.getRotation().getZ()) * cameraToTagDistanceMeters;
        return new Translation2d(cameraToTagXDistance, cameraToTagYDistance);
    }

    private Pose2d getRobotPoseFromCameraPose(Pose3d solvePNPPose) {
        if (solvePNPPose == null)
            return null;
        return solvePNPPose.transformBy(robotCenterToCamera.inverse()).toPose2d();
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLastResultTimestamp())
            return false;

        lastUpdatedTimestamp = getLastResultTimestamp();
        return true;
    }

    private void logVisibleTags() {
        if (inputs.hasResult) {
            Logger.recordOutput("VisibleTags/" + this.getName(), new Pose2d[0]);
            return;
        }

        final Pose2d[] visibleTagPoses = new Pose2d[inputs.visibleTagIDs.length];
        for (int i = 0; i < visibleTagPoses.length; i++) {
            visibleTagPoses[i] = FieldConstants.TAG_ID_TO_POSE.get(i).toPose2d();
        }
        Logger.recordOutput("VisibleTags/" + this.getName(), visibleTagPoses);
    }
}
