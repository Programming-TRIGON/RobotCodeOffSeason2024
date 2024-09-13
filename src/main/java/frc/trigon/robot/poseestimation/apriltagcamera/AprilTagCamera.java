package frc.trigon.robot.poseestimation.apriltagcamera;

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
 * An april tag camera is a class that provides the robot's pose, from a camera using one or multiple apriltags.
 * An april tag is like a 2D barcode used to find the robot's position on the field.
 * Since the tag's position on the field is known, we can calculate our position relative to it, therefore estimating our position on the field.
 */
public class AprilTagCamera {
    protected final String name;
    private final RobotPoseSourceInputsAutoLogged inputs = new RobotPoseSourceInputsAutoLogged();
    private final Transform3d robotCenterToCamera;
    private final AprilTagCameraIO aprilTagCameraIO;
    private double lastUpdatedTimestamp;
    private final double solvePNPTranslationsStandardDeviationsExponent, assumedRobotPoseTranslationsStandardDeviationsExponent, solvePNPThetaStandardDeviationsExponent;
    private Pose2d robotPose = null;

    /**
     * Constructs a new AprilTagCamera.
     *
     * @param robotPoseSourceType                                       the type of camera
     * @param name                                                      the camera's name
     * @param robotCenterToCamera                                       the transform of the robot's origin point to the camera
     * @param solvePNPTranslationsStandardDeviationsExponent            the calibrated gain to calculate the translation deviation from the estimated pose when using solve PNP
     * @param solvePNPThetaStandardDeviationsExponent                   the calibrated gain to calculate the theta deviation from the estimated pose when using solve PNP
     * @param assumedRobotHeadingTranslationsStandardDeviationsExponent the calibrated gain to calculate the translation deviation from the estimated pose when getting the pose by assuming the robot's heading
     */
    public AprilTagCamera(AprilTagCameraConstants.RobotPoseSourceType robotPoseSourceType, String name, Transform3d robotCenterToCamera, double solvePNPTranslationsStandardDeviationsExponent, double solvePNPThetaStandardDeviationsExponent, double assumedRobotHeadingTranslationsStandardDeviationsExponent) {
        this.name = name;
        this.robotCenterToCamera = robotCenterToCamera;
        this.solvePNPTranslationsStandardDeviationsExponent = solvePNPTranslationsStandardDeviationsExponent;
        this.solvePNPThetaStandardDeviationsExponent = solvePNPThetaStandardDeviationsExponent;
        this.assumedRobotPoseTranslationsStandardDeviationsExponent = assumedRobotHeadingTranslationsStandardDeviationsExponent;

        if (Robot.IS_REAL)
            aprilTagCameraIO = robotPoseSourceType.createIOFunction.apply(name);
        else
            aprilTagCameraIO = new AprilTagCameraIO();
    }

    public void update() {
        aprilTagCameraIO.updateInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);

        robotPose = calculateBestRobotPose(inputs.distanceFromBestTag);

        if (!inputs.hasResult || inputs.averageDistanceFromAllTags == 0 || robotPose == null)
            Logger.recordOutput("Poses/Robot/" + name + "Pose", AprilTagCameraConstants.EMPTY_POSE_LIST);
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
     * Calculates the range of how inaccurate the estimated pose could be using the distance from the target, the number of targets, and a calculated gain.
     * Different pose estimation strategies may use different formulae to calculate the standard deviations.
     *
     * @return the standard deviations for the pose estimation strategy used
     */
    public Matrix<N3, N1> calculateStandardDeviations() {
        if (isWithinBestTagRangeForSolvePNP())
            return calculateSolvePNPStandardDeviations();
        return calculateAssumedHeadingStandardDeviations();
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
        final Rotation2d robotHeadingAtResultTimestamp = PoseEstimator6328.getInstance().samplePose(inputs.lastResultTimestamp).getRotation();
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

    private Matrix<N3, N1> calculateSolvePNPStandardDeviations() {
        final int numberOfVisibleTags = inputs.visibleTagIDs.length;
        final double translationStandardDeviation = calculateStandardDeviations(solvePNPTranslationsStandardDeviationsExponent, inputs.averageDistanceFromAllTags, numberOfVisibleTags);
        final double thetaStandardDeviation = calculateStandardDeviations(solvePNPThetaStandardDeviationsExponent, inputs.averageDistanceFromAllTags, numberOfVisibleTags);

        return VecBuilder.fill(translationStandardDeviation, translationStandardDeviation, thetaStandardDeviation);
    }

    /**
     * Calculates the standard deviations of the pose, for when the pose is calculated by assuming the robot's heading.
     * The theta deviation is infinite so that it won't change anything in the heading because we assume it is correct.
     *
     * @return the standard deviations
     */
    private Matrix<N3, N1> calculateAssumedHeadingStandardDeviations() {
        final double translationStandardDeviation = calculateStandardDeviations(assumedRobotPoseTranslationsStandardDeviationsExponent, inputs.distanceFromBestTag, inputs.visibleTagIDs.length);
        final double thetaStandardDeviation = Double.POSITIVE_INFINITY;

        return VecBuilder.fill(translationStandardDeviation, translationStandardDeviation, thetaStandardDeviation);
    }

    /**
     * Calculates the standard deviation of the estimated pose using a formula.
     * As we get further from the tag(s), this will return a less trusting (higher deviation) result.
     *
     * @param exponent            a calibrated gain, different for each pose estimating strategy
     * @param distance            the distance from the tag(s)
     * @param numberOfVisibleTags the number of visible tags
     * @return the standard deviation
     */
    private double calculateStandardDeviations(double exponent, double distance, int numberOfVisibleTags) {
        return exponent * (distance * distance) / numberOfVisibleTags;
    }

    private boolean isWithinBestTagRangeForSolvePNP() {
        return inputs.distanceFromBestTag < AprilTagCameraConstants.MAXIMUM_DISTANCE_FROM_TAG_FOR_PNP_METERS;
    }
}