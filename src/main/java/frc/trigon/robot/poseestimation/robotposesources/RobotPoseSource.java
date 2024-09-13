package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.trigon.robot.Robot;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator6328;
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
    private final double solvePNPTranslationsStdExponent, assumedRobotPoseTranslationsStdExponent, thetaStdExponent;
    private Pose2d robotPose = null;

    public RobotPoseSource(RobotPoseSourceConstants.RobotPoseSourceType robotPoseSourceType, String name, Transform3d robotCenterToCamera, double solvePNPTranslationsStdExponent, double assumedRobotPoseTranslationsStdExponent, double thetaStdExponent) {
        this.name = name;
        this.robotCenterToCamera = robotCenterToCamera;
        this.solvePNPTranslationsStdExponent = solvePNPTranslationsStdExponent;
        this.assumedRobotPoseTranslationsStdExponent = assumedRobotPoseTranslationsStdExponent;
        this.thetaStdExponent = thetaStdExponent;

        if (Robot.IS_REAL)
            robotPoseSourceIO = robotPoseSourceType.createIOFunction.apply(name);
        else
            robotPoseSourceIO = new RobotPoseSourceIO();
    }

    public void update() {
        robotPoseSourceIO.updateInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);

        robotPose = calculateBestRobotPose(inputs.distanceFromBestTag);

        if (!inputs.hasResult || inputs.averageDistanceFromAllTags == 0 || robotPose == null)
            Logger.recordOutput("Poses/Robot/" + name + "Pose", RobotPoseSourceConstants.EMPTY_POSE_LIST);
        else
            Logger.recordOutput("Poses/Robot/" + name + "Pose", robotPose);
    }

    public boolean hasNewResult() {
        return (inputs.hasResult && inputs.averageDistanceFromAllTags != 0) && isNewTimestamp();
    }

    public Pose2d getEstimatedRobotPose() {
        return robotPose;
    }

    public String getName() {
        return name;
    }

    public double getLastResultTimestamp() {
        return inputs.lastResultTimestamp;
    }


    /**
     * Calculates the range of how inaccurate the estimated pose could be
     *
     * @return the standard deviations for the pose estimation strategy used
     */
    public Matrix<N3, N1> calculateStdDevs() {
        if (isWithinBestTagRangeForSolvePNP())
            return calculateSolvePNPStdDevs();
        return calculateAssumedHeadingStdDevs();
    }

    /**
     * If the robot is close enough to the tag, it gets the estimated solve PNP pose.
     * If it's too far, it assumes the robot's heading and calculates its position from there.
     * Assuming the robot's heading is more robust, but won't fix current wrong heading.
     * To fix this, we use solve PNP to reset the robot's heading, when we're close enough for an accurate result.
     *
     * @param distanceFromBestTag the average of the distance from the best visible tag
     * @return the robot's pose
     */
    private Pose2d calculateBestRobotPose(double distanceFromBestTag) {
        if (isWithinBestTagRangeForSolvePNP())
            return cameraPoseToRobotPose(inputs.solvePNPPose);
        return calculateAssumedRobotHeadingPose().toPose2d();
    }

    /**
     * Calculates the robot's pose by assuming its heading, the apriltag's position, and the camera's position on the robot.
     * This method of finding the robot's pose is more robust but relies on knowing the robot's heading beforehand.
     *
     * @return the robot's pose
     */
    private Pose3d calculateAssumedRobotHeadingPose() {
        final Rotation2d robotHeadingAtResultTimestamp = PoseEstimator6328.getInstance().getSamplePose(inputs.lastResultTimestamp).getRotation();
        final Translation2d robotFieldRelativePositionTranslation = getRobotFieldRelativePosition(robotHeadingAtResultTimestamp);
        return new Pose3d(new Pose2d(robotFieldRelativePositionTranslation, robotHeadingAtResultTimestamp));
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
                robotCenterToCamera.getZ(), tagPose.getZ(), robotCenterToCamera.getRotation().getY(), inputs.bestTargetRelativePitchRadians
        );
        final double cameraToTagXDistance = Math.sin(inputs.bestTargetRelativeYawRadians + robotCenterToCamera.getRotation().getZ()) * cameraToTagDistanceMeters;
        final double cameraToTagYDistance = -Math.cos(inputs.bestTargetRelativeYawRadians + robotCenterToCamera.getRotation().getZ()) * cameraToTagDistanceMeters;
        return new Translation2d(cameraToTagXDistance, cameraToTagYDistance);
    }

    private Pose2d cameraPoseToRobotPose(Pose3d cameraPose) {
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

    private void logVisibleTags() {
        if (inputs.hasResult) {
            Logger.recordOutput("VisibleTags/" + this.getName(), new Pose2d[0]);
            return;
        }

        final Pose2d[] visibleTagPoses = new Pose2d[inputs.visibleTagIDs.length];
        for (int i = 0; i < visibleTagPoses.length; i++)
            visibleTagPoses[i] = FieldConstants.TAG_ID_TO_POSE.get(i).toPose2d();
        Logger.recordOutput("VisibleTags/" + this.getName(), visibleTagPoses);
    }

    private Matrix<N3, N1> calculateSolvePNPStdDevs() {
        final int numberOfVisibleTags = inputs.visibleTagIDs.length;
        final double translationStd = solvePNPTranslationsStdExponent * inputs.averageDistanceFromAllTags * inputs.averageDistanceFromAllTags / numberOfVisibleTags;
        final double thetaStd = thetaStdExponent * Math.pow(inputs.averageDistanceFromAllTags, 2) / numberOfVisibleTags;

        return VecBuilder.fill(translationStd, translationStd, thetaStd);
    }

    /**
     * Calculates the standard deviations of the pose, for when the pose is calculated by assuming the robot's heading.
     * The theta deviation is infinite so that it won't change anything in the heading because we assume it is correct.
     *
     * @return the standard deviations
     */
    private Matrix<N3, N1> calculateAssumedHeadingStdDevs() {
        final double translationStd = assumedRobotPoseTranslationsStdExponent * inputs.distanceFromBestTag * inputs.distanceFromBestTag;
        final double thetaStd = Double.POSITIVE_INFINITY;

        return VecBuilder.fill(translationStd, translationStd, thetaStd);
    }

    private boolean isWithinBestTagRangeForSolvePNP() {
        return inputs.distanceFromBestTag < RobotPoseSourceConstants.MAXIMUM_DISTANCE_FROM_TAG_FOR_PNP_METERS;
    }
}
