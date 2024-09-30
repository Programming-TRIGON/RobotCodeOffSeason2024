package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.trigon.robot.Robot;
import frc.trigon.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

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
    private final double
            thetaStandardDeviationsExponent,
            translationStandardDeviationsExponent;
    private double lastUpdatedTimestamp;
    private Pose2d robotPose = null;

    /**
     * Constructs a new AprilTagCamera.
     *
     * @param robotPoseSourceType                                      the type of camera
     * @param name                                                     the camera's name
     * @param robotCenterToCamera                                      the transform of the robot's origin point to the camera
     * @param thetaStandardDeviationsExponent                  the calibrated gain to calculate the theta deviation from the estimated pose when using solve PNP
     * @param translationStandardDeviationsExponent            the calibrated gain to calculate the translation deviation from the estimated pose when using solve PNP
     */
    public AprilTagCamera(AprilTagCameraConstants.RobotPoseSourceType robotPoseSourceType, String name, Transform3d robotCenterToCamera, double thetaStandardDeviationsExponent, double translationStandardDeviationsExponent) {
        this.name = name;
        this.robotCenterToCamera = robotCenterToCamera;
        this.thetaStandardDeviationsExponent = thetaStandardDeviationsExponent;
        this.translationStandardDeviationsExponent = translationStandardDeviationsExponent;

        if (Robot.IS_REAL)
            aprilTagCameraIO = robotPoseSourceType.createIOFunction.apply(name);
        else
            aprilTagCameraIO = new AprilTagCameraIO();
    }

    public void update() {
        aprilTagCameraIO.updateInputs(inputs);
        Logger.processInputs("Cameras/" + name, inputs);
        robotPose = inputs.cameraSolvePNPPose.transformBy(robotCenterToCamera.inverse()).toPose2d();

        logEstimatedRobotPose();
        if (!FieldConstants.TAG_ID_TO_POSE.isEmpty())
            logUsedTags();
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

    public double getLatestResultTimestampSeconds() {
        return inputs.latestResultTimestampSeconds;
    }

    private boolean isNewTimestamp() {
        if (lastUpdatedTimestamp == getLatestResultTimestampSeconds())
            return false;

        lastUpdatedTimestamp = getLatestResultTimestampSeconds();
        return true;
    }

    public Matrix<N3, N1> getStandardDeviations() {
        final int numberOfVisibleTags = inputs.visibleTagIDs.length;
        final double translationStandardDeviation = calculateStandardDeviations(translationStandardDeviationsExponent, inputs.averageDistanceFromAllTags, numberOfVisibleTags);
        final double thetaStandardDeviation = calculateStandardDeviations(thetaStandardDeviationsExponent, inputs.averageDistanceFromAllTags, numberOfVisibleTags);

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

    private void logEstimatedRobotPose() {
        if (!inputs.hasResult || inputs.averageDistanceFromAllTags == 0 || robotPose == null)
            Logger.recordOutput("Poses/Robot/" + name + "Pose", AprilTagCameraConstants.EMPTY_POSE_LIST);
        else
            Logger.recordOutput("Poses/Robot/" + name + "Pose", robotPose);
    }

    private void logUsedTags() {
        if (!inputs.hasResult) {
            Logger.recordOutput("UsedTags/" + this.getName(), new Pose3d[0]);
            return;
        }

        final Pose3d[] usedTagPoses = new Pose3d[inputs.visibleTagIDs.length];
        for (int i = 0; i < usedTagPoses.length; i++)
            usedTagPoses[i] = FieldConstants.TAG_ID_TO_POSE.get(inputs.visibleTagIDs[i]);
        Logger.recordOutput("UsedTags/" + this.getName(), usedTagPoses);
    }
}