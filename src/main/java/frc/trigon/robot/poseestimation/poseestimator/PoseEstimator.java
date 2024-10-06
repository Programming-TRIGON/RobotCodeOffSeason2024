package frc.trigon.robot.poseestimation.poseestimator;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

/**
 * A class that estimates the robot's pose using team 6328's custom pose estimator.
 */
public class PoseEstimator implements AutoCloseable {
    private final Field2d field = new Field2d();
    private final AprilTagCamera[] aprilTagCameras;
    private final PoseEstimator6328 poseEstimator6328 = PoseEstimator6328.getInstance();

    /**
     * Constructs a new PoseEstimator.
     *
     * @param aprilTagCameras the sources that should update the pose estimator apart from the odometry. This should be cameras etc.
     */
    public PoseEstimator(AprilTagCamera... aprilTagCameras) {
        this.aprilTagCameras = aprilTagCameras;
        putAprilTagsOnFieldWidget();
        SmartDashboard.putData("Field", field);
        PathPlannerLogging.setLogActivePathCallback((pose) -> {
            field.getObject("path").setPoses(pose);
            Logger.recordOutput("Path", pose.toArray(new Pose2d[0]));
        });
    }

    @Override
    public void close() {
        field.close();
    }

    public void periodic() {
        updateFromVision();
        field.setRobotPose(getCurrentPose());
    }

    /**
     * Resets the pose estimator to the given pose, and the gyro to the given pose's heading.
     *
     * @param currentPose the pose to reset to, relative to the blue alliance's driver station right corner
     */
    public void resetPose(Pose2d currentPose) {
        RobotContainer.SWERVE.setHeading(currentPose.getRotation());
        poseEstimator6328.resetPose(currentPose);
    }

    /**
     * @return the estimated pose of the robot, relative to the blue alliance's driver station right corner
     */
    public Pose2d getCurrentPose() {
        return poseEstimator6328.getEstimatedPose();
    }

    /**
     * Updates the pose estimator with the given SWERVE wheel positions and gyro rotations.
     * This function accepts an array of SWERVE wheel positions and an array of gyro rotations because the odometry can be updated at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with all of them.
     *
     * @param swerveWheelPositions the SWERVE wheel positions accumulated since the last update
     * @param gyroRotations        the gyro rotations accumulated since the last update
     */
    public void updatePoseEstimatorStates(SwerveDriveWheelPositions[] swerveWheelPositions, Rotation2d[] gyroRotations, double[] timestamps) {
        for (int i = 0; i < swerveWheelPositions.length; i++)
            poseEstimator6328.addOdometryObservation(new PoseEstimator6328.OdometryObservation(swerveWheelPositions[i], gyroRotations[i], timestamps[i]));
    }

    public void setGyroHeadingToBestSolvePNPHeading() {
        int closestCameraToTag = 0;
        for (int i = 0; i < aprilTagCameras.length; i++) {
            if (aprilTagCameras[i].getDistanceToBestTagMeters() < aprilTagCameras[closestCameraToTag].getDistanceToBestTagMeters())
                closestCameraToTag = i;
        }

        final Rotation2d bestRobotHeading = aprilTagCameras[closestCameraToTag].getSolvePNPHeading();
        resetPose(new Pose2d(getCurrentPose().getTranslation(), bestRobotHeading));
    }

    private void updateFromVision() {
        getViableVisionObservations().stream()
                .sorted(Comparator.comparingDouble(PoseEstimator6328.VisionObservation::timestamp))
                .forEach(poseEstimator6328::addVisionObservation);
    }

    private List<PoseEstimator6328.VisionObservation> getViableVisionObservations() {
        List<PoseEstimator6328.VisionObservation> viableVisionObservations = new ArrayList<>();
        for (AprilTagCamera aprilTagCamera : aprilTagCameras) {
            final PoseEstimator6328.VisionObservation visionObservation = getVisionObservation(aprilTagCamera);
            if (visionObservation != null)
                viableVisionObservations.add(visionObservation);
        }
        return viableVisionObservations;
    }

    private PoseEstimator6328.VisionObservation getVisionObservation(AprilTagCamera aprilTagCamera) {
        aprilTagCamera.update();
        if (!aprilTagCamera.hasNewResult())
            return null;
        final Pose2d robotPose = aprilTagCamera.getEstimatedRobotPose();
        if (robotPose == null)
            return null;

        return new PoseEstimator6328.VisionObservation(
                robotPose,
                aprilTagCamera.getLatestResultTimestampSeconds(),
                aprilTagCamera.calculateStandardDeviations()
        );
    }

    private void putAprilTagsOnFieldWidget() {
        for (Map.Entry<Integer, Pose3d> entry : FieldConstants.TAG_ID_TO_POSE.entrySet()) {
            final Pose2d tagPose = entry.getValue().toPose2d();
            field.getObject("Tag " + entry.getKey()).setPose(tagPose);
        }
    }
}
