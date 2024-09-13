package frc.trigon.robot.poseestimation.apriltagcamera.io;

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
    public void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.hasResult = LimelightHelpers.getTV(hostname);
        if (inputs.hasResult) {
            final LimelightHelpers.Results results = LimelightHelpers.getLatestResults(hostname).targetingResults;
            inputs.solvePNPPose = results.getBotPose3d_wpiBlue();
            inputs.lastResultTimestamp = results.timestamp_LIMELIGHT_publish;
            inputs.visibleTagIDs = getVisibleTagIDs(results);
            inputs.bestTargetRelativeYawRadians = results.getBotPose3d_wpiBlue().getRotation().getZ();
            inputs.bestTargetRelativePitchRadians = results.getBotPose3d_wpiBlue().getRotation().getY();
            inputs.averageDistanceFromAllTags = getAverageDistanceFromAllTags(results);
            inputs.distanceFromBestTag = getDistanceFromBestTag(results);
        }
    }

    private int[] getVisibleTagIDs(LimelightHelpers.Results results) {
        LimelightHelpers.LimelightTarget_Fiducial[] visibleTags = results.targets_Fiducials;
        int[] visibleTagIDs = new int[visibleTags.length];
        for (int i = 0; i < visibleTagIDs.length; i++)
            visibleTagIDs[i] = (int) visibleTags[i].fiducialID;
        return visibleTagIDs;
    }

    private double getAverageDistanceFromAllTags(LimelightHelpers.Results results) {
        int[] visibleTags = getVisibleTagIDs(results);
        double totalDistanceFromTags = 0;
        for (int i = 0; i < visibleTags.length; i++)
            totalDistanceFromTags += FieldConstants.TAG_ID_TO_POSE.get(i).getTranslation().getDistance(results.getBotPose3d_wpiBlue().getTranslation());
        return totalDistanceFromTags /= 4;
    }

    private double getDistanceFromBestTag(LimelightHelpers.Results results) {
        return FieldConstants.TAG_ID_TO_POSE.get(getVisibleTagIDs(results)[0]).getTranslation().getDistance(results.getBotPose3d_wpiBlue().getTranslation());
    }
}