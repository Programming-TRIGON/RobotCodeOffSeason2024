package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Pose3d;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.RobotPoseSourceInputsAutoLogged;
import org.trigon.utilities.LimelightHelpers;

public class AprilTagLimelightIO extends AprilTagCameraIO {
    private final String hostname;

    public AprilTagLimelightIO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        final LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(hostname);

        inputs.hasResult = results.targets_Fiducials.length > 0;

        if (inputs.hasResult)
            updateHasResultInputs(inputs, results);
        else
            updateNoResultInputs(inputs);
    }

    private void updateHasResultInputs(RobotPoseSourceInputsAutoLogged inputs, LimelightHelpers.LimelightResults results) {
        inputs.cameraSolvePNPPose = results.getBotPose3d_wpiBlue();
        inputs.latestResultTimestampSeconds = results.timestamp_RIOFPGA_capture;
        inputs.visibleTagIDs = getVisibleTagIDs(results);
        inputs.averageDistanceFromAllTags = getAverageDistanceFromAllTags(results);
        inputs.distanceFromBestTag = getDistanceFromBestTag(results);
    }

    private void updateNoResultInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.visibleTagIDs = new int[0];
        inputs.cameraSolvePNPPose = new Pose3d();
    }

    private int[] getVisibleTagIDs(LimelightHelpers.LimelightResults results) {
        final LimelightHelpers.LimelightTarget_Fiducial[] visibleTags = results.targets_Fiducials;
        final int[] visibleTagIDs = new int[visibleTags.length];
        visibleTagIDs[0] = (int) getBestTarget(results).fiducialID;
        int idAddition = 1;

        for (int i = 0; i < visibleTagIDs.length; i++) {
            final int currentID = (int) visibleTags[i].fiducialID;

            if (currentID == visibleTagIDs[0]) {
                idAddition = 0;
                continue;
            }

            visibleTagIDs[i + idAddition] = currentID;
        }
        return visibleTagIDs;
    }

    private double getAverageDistanceFromAllTags(LimelightHelpers.LimelightResults results) {
        final LimelightHelpers.LimelightTarget_Fiducial[] targetFiducials = results.targets_Fiducials;
        double totalDistanceFromTags = 0;

        for (LimelightHelpers.LimelightTarget_Fiducial targetFiducial : targetFiducials)
            totalDistanceFromTags += getDistanceFromTag((int) targetFiducial.fiducialID, results.getBotPose3d_wpiBlue());

        return totalDistanceFromTags / results.targets_Fiducials.length;
    }

    private double getDistanceFromBestTag(LimelightHelpers.LimelightResults results) {
        return getDistanceFromTag((int) getBestTarget(results).fiducialID, results.getBotPose3d_wpiBlue());
    }

    private double getDistanceFromTag(int fiducialID, Pose3d estimatedRobotPose) {
        return FieldConstants.TAG_ID_TO_POSE.get(fiducialID).getTranslation().getDistance(estimatedRobotPose.getTranslation());
    }

    private LimelightHelpers.LimelightTarget_Fiducial getBestTarget(LimelightHelpers.LimelightResults results) {
        return results.targets_Fiducials[0];
    }
}