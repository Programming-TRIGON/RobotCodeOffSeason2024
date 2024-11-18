package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.poseestimator.PoseEstimator6328;
import org.littletonrobotics.junction.Logger;
import org.trigon.hardware.RobotHardwareStats;

/**
 * An april tag camera is a class that provides the robot's pose, from a camera using one or multiple apriltags.
 * An april tag is like a 2D barcode used to find the robot's position on the field.
 * Since the tag's position on the field is known, we can calculate our position relative to it, therefore estimating our position on the field.
 */
public class AprilTagCamera {
    protected final String name;
    private final AprilTagCameraInputsAutoLogged inputs = new AprilTagCameraInputsAutoLogged();
    private final Transform3d robotCenterToCamera;
    private final double
            thetaStandardDeviationExponent,
            translationStandardDeviationExponent;
    private final AprilTagCameraIO aprilTagCameraIO;
    private double lastUpdatedTimestamp;
    private Pose2d robotPose = null;

    /**
     * Constructs a new AprilTagCamera.
     *
     * @param aprilTagCameraType                   the type of camera
     * @param name                                 the camera's name
     * @param robotCenterToCamera                  the transform of the robot's origin point to the camera
     * @param thetaStandardDeviationExponent       a calibrated gain for the standard theta deviations of the estimated robot pose
     * @param translationStandardDeviationExponent a calibrated gain for the standard translation deviations of the estimated robot pose
     */
    public AprilTagCamera(AprilTagCameraConstants.AprilTagCameraType aprilTagCameraType,
                          String name, Transform3d robotCenterToCamera,
                          double thetaStandardDeviationExponent,
                          double translationStandardDeviationExponent) {
        this.name = name;
        this.robotCenterToCamera = robotCenterToCamera;
        this.thetaStandardDeviationExponent = thetaStandardDeviationExponent;
        this.translationStandardDeviationExponent = translationStandardDeviationExponent;

        if (RobotHardwareStats.isSimulation()) {
            aprilTagCameraIO = AprilTagCameraConstants.AprilTagCameraType.SIMULATION_CAMERA.createIOFunction.apply(name);
            aprilTagCameraIO.addSimulatedCameraToVisionSimulation(robotCenterToCamera);
            return;
        }
        aprilTagCameraIO = aprilTagCameraType.createIOFunction.apply(name);
    }

    public void update() {
        aprilTagCameraIO.updateInputs(inputs);

        robotPose = calculateBestRobotPose();
        logCameraInfo();
        if (RobotHardwareStats.isSimulation())
            AprilTagCameraConstants.VISION_SIMULATION.update(RobotContainer.POSE_ESTIMATOR.getCurrentPose());
    }

    public boolean hasNewResult() {
        return (inputs.hasResult && inputs.distanceFromBestTag != 0) && isNewTimestamp();
    }

    public Pose2d getEstimatedRobotPose() {
        return robotPose;
    }

    public Rotation2d getSolvePNPHeading() {
        return inputs.cameraSolvePNPPose.getRotation().toRotation2d().minus(robotCenterToCamera.getRotation().toRotation2d());
    }

    public String getName() {
        return name;
    }

    public double getLatestResultTimestampSeconds() {
        return inputs.latestResultTimestampSeconds;
    }

    /**
     * Calculates the range of how inaccurate the estimated pose could be using the distance from the target, the number of targets, and a calibrated gain.
     * The theta deviation is infinity when we assume the robot's heading because we already assume that the heading is correct.
     *
     * @return the standard deviations for the pose estimation strategy used
     */
    public Matrix<N3, N1> calculateStandardDeviations() {
        final double translationStandardDeviation = calculateStandardDeviations(translationStandardDeviationExponent, inputs.distanceFromBestTag, inputs.visibleTagIDs.length);
        final double thetaStandardDeviation = isWithinBestTagRangeForSolvePNP() ? calculateStandardDeviations(thetaStandardDeviationExponent, inputs.distanceFromBestTag, inputs.visibleTagIDs.length) : Double.POSITIVE_INFINITY;

        return VecBuilder.fill(translationStandardDeviation, translationStandardDeviation, thetaStandardDeviation);
    }

    public double getDistanceToBestTagMeters() {
        return inputs.distanceFromBestTag;
    }

    /**
     * If the robot is close enough to the tag, it calculates the pose using the solve PNP heading.
     * If it's too far, it assumes the robot's heading from the gyro and calculates its position from there.
     * Assuming the robot's heading from the gyro is more robust, but won't fix current wrong heading.
     * To fix this, we use solve PNP to reset the gyro when we are close enough for an accurate result.
     *
     * @return the robot's pose
     */
    private Pose2d calculateBestRobotPose() {
        final Rotation2d gyroHeadingAtTimestamp = PoseEstimator6328.getInstance().samplePose(inputs.latestResultTimestampSeconds).getRotation();
        return calculateAssumedRobotHeadingPose(gyroHeadingAtTimestamp);
    }

    /**
     * Calculates the robot's pose by assuming its heading, the apriltag's position, and the camera's position on the robot.
     * This method of pose estimation is more robust than solve PNP, but relies on knowing the robot's heading beforehand.
     *
     * @return the robot's pose
     */
    private Pose2d calculateAssumedRobotHeadingPose(Rotation2d gyroHeading) {
        if (inputs.visibleTagIDs.length == 0 || !inputs.hasResult || inputs.poseAmbiguity > AprilTagCameraConstants.MAXIMUM_AMBIGUITY)
            return null;

        if (!isWithinBestTagRangeForSolvePNP())
            return new Pose2d(getFieldRelativeRobotTranslation(gyroHeading), gyroHeading);
        final Rotation2d solvePNPHeading = getSolvePNPHeading();
        return new Pose2d(getFieldRelativeRobotTranslation(solvePNPHeading), solvePNPHeading);
    }

    private Translation2d getFieldRelativeRobotTranslation(Rotation2d currentHeading) {
        final Pose3d bestTagPose = FieldConstants.TAG_ID_TO_POSE.get(inputs.visibleTagIDs[0]);
        if (bestTagPose == null)
            return null;

        setProperCameraRotation();

        final Translation2d tagRelativeCameraTranslation = calculateTagRelativeCameraTranslation(currentHeading, bestTagPose);
        final Translation2d fieldRelativeRobotPose = getFieldRelativeRobotPose(tagRelativeCameraTranslation, bestTagPose);
        final Translation2d fieldRelativeCameraToRobotTranslation = robotCenterToCamera.getTranslation().toTranslation2d().rotateBy(currentHeading);
        return fieldRelativeRobotPose.minus(fieldRelativeCameraToRobotTranslation);
    }

    /**
     * When the roll of the camera changes, the target pitch and yaw are also effected.
     * This method corrects the yaw and pitch based on the camera's mount position roll.
     */
    private void setProperCameraRotation() {
        Translation2d targetRotationVector = new Translation2d(inputs.bestTargetRelativeYawRadians, inputs.bestTargetRelativePitchRadians);
        targetRotationVector.rotateBy(Rotation2d.fromRadians(robotCenterToCamera.getRotation().getX()));
        inputs.bestTargetRelativeYawRadians = targetRotationVector.getY();
        inputs.bestTargetRelativePitchRadians = targetRotationVector.getX();
    }

    private Translation2d calculateTagRelativeCameraTranslation(Rotation2d gyroHeading, Pose3d tagPose) {
        final double robotPlaneTargetYawRadians = getRobotPlaneTargetYawRadians();
        final double robotPlaneCameraDistanceToUsedTagMeters = calculateRobotPlaneXYDistanceToTag(tagPose, robotPlaneTargetYawRadians);
        final double headingOffsetToUsedTagRadians = gyroHeading.getRadians() - robotPlaneTargetYawRadians + robotCenterToCamera.getRotation().getZ();
        return new Translation2d(robotPlaneCameraDistanceToUsedTagMeters, Rotation2d.fromRadians(headingOffsetToUsedTagRadians));
    }

    private double getRobotPlaneTargetYawRadians() {
        double targetYawRadians = -inputs.bestTargetRelativeYawRadians;
        for (int i = 0; i < AprilTagCameraConstants.CALCULATE_YAW_ITERATIONS; i++) {
            final double projectedPitch = projectToPlane(-robotCenterToCamera.getRotation().getY(), targetYawRadians + Math.PI / 2);
            targetYawRadians = -inputs.bestTargetRelativeYawRadians - Math.tan(projectedPitch) * -inputs.bestTargetRelativePitchRadians;
        }
        return projectToPlane(targetYawRadians, robotCenterToCamera.getRotation().getY());
    }

    private double projectToPlane(double targetAngleRadians, double cameraAngleRadians) {
        if (cameraAngleRadians < 0)
            return Math.atan(Math.tan(targetAngleRadians) / Math.cos(cameraAngleRadians));
        return Math.atan(Math.tan(targetAngleRadians) * Math.cos(cameraAngleRadians));
    }

    private double calculateRobotPlaneXYDistanceToTag(Pose3d usedTagPose, double robotPlaneTargetYaw) {
        final double zDistanceToUsedTagMeters = Math.abs(usedTagPose.getZ() - robotCenterToCamera.getTranslation().getZ());
        final double robotPlaneDistanceFromUsedTagMeters = zDistanceToUsedTagMeters / Math.tan(-robotCenterToCamera.getRotation().getY() - inputs.bestTargetRelativePitchRadians);
        return robotPlaneDistanceFromUsedTagMeters / Math.cos(robotPlaneTargetYaw);
    }

    private Translation2d getFieldRelativeRobotPose(Translation2d tagRelativeCameraTranslation, Pose3d tagPose) {
        return tagPose.getTranslation().toTranslation2d().minus(tagRelativeCameraTranslation);
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLatestResultTimestampSeconds())
            return false;

        lastUpdatedTimestamp = getLatestResultTimestampSeconds();
        return true;
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
        return inputs.distanceFromBestTag < AprilTagCameraConstants.MAXIMUM_DISTANCE_FROM_TAG_FOR_SOLVE_PNP_METERS;
    }

    private void logCameraInfo() {
        Logger.processInputs("Cameras/" + name, inputs);
        if (!FieldConstants.TAG_ID_TO_POSE.isEmpty())
            logUsedTags();
        if (!inputs.hasResult || inputs.distanceFromBestTag == 0 || robotPose == null) {
            logEstimatedRobotPose();
            logSolvePNPPose();
        } else {
            Logger.recordOutput("Poses/Robot/" + name + "/Pose", AprilTagCameraConstants.EMPTY_POSE_LIST);
            Logger.recordOutput("Poses/Robot/" + name + "/SolvePNPPose", AprilTagCameraConstants.EMPTY_POSE_LIST);
        }
    }

    private void logUsedTags() {
        if (!inputs.hasResult) {
            Logger.recordOutput("UsedTags/" + this.getName(), new Pose3d[0]);
            return;
        }

        final Pose3d[] usedTagPoses = isWithinBestTagRangeForSolvePNP() ? new Pose3d[inputs.visibleTagIDs.length] : new Pose3d[1];
        for (int i = 0; i < usedTagPoses.length; i++)
            usedTagPoses[i] = FieldConstants.TAG_ID_TO_POSE.get(inputs.visibleTagIDs[i]);
        Logger.recordOutput("UsedTags/" + this.getName(), usedTagPoses);
    }

    private void logEstimatedRobotPose() {
        Logger.recordOutput("Poses/Robot/" + name + "/Pose", robotPose);
    }

    private void logSolvePNPPose() {
        Logger.recordOutput("Poses/Robot/" + name + "/SolvePNPPose", inputs.cameraSolvePNPPose.plus(robotCenterToCamera.inverse()));
    }
}