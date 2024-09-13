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
        inputs.hasResult = LimelightHelpers.getTV(hostname);

        if (inputs.hasResult)
            updateHasResultInputs(inputs);
        else
            updateNoResultInputs(inputs);
    }

    private void updateHasResultInputs(RobotPoseSourceInputsAutoLogged inputs) {
        final LimelightHelpers.Results results = LimelightHelpers.getLatestResults(hostname).targetingResults;
        inputs.solvePNPPose = results.getBotPose3d_wpiBlue();
        inputs.lastResultTimestamp = results.timestamp_LIMELIGHT_publish;
        inputs.visibleTagIDs = getVisibleTagIDs(results);
//        inputs.bestTargetRelativeYawRadians =;
//        inputs.bestTargetRelativePitchRadians =;
        inputs.averageDistanceFromAllTags = getAverageDistanceFromAllTags(results);
        inputs.distanceFromBestTag = getDistanceFromBestTag(results);
    }

    private void updateNoResultInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.visibleTagIDs = new int[0];
        inputs.solvePNPPose = new Pose3d();
    }

    private int[] getVisibleTagIDs(LimelightHelpers.Results results) {
        final LimelightHelpers.LimelightTarget_Fiducial[] visibleTags = results.targets_Fiducials;
        final int[] visibleTagIDs = new int[visibleTags.length];
        for (int i = 0; i < visibleTagIDs.length; i++)
            visibleTagIDs[i] = (int) visibleTags[i].fiducialID;
        return visibleTagIDs;
    }

    private double getAverageDistanceFromAllTags(LimelightHelpers.Results results) {
        double totalDistanceFromTags = 0;
        for (int i = 0; i < results.targets_Fiducials.length; i++)
            totalDistanceFromTags += getDistanceFromTag(i, results.getBotPose3d_wpiBlue());
        return totalDistanceFromTags /= results.targets_Fiducials.length;
    }

    private double getDistanceFromBestTag(LimelightHelpers.Results results) {
        return getDistanceFromTag((int) results.targets_Fiducials[0].fiducialID, results.getBotPose3d_wpiBlue());
    }

    private double getDistanceFromTag(int fiducialID, Pose3d robotPose) {
        return FieldConstants.TAG_ID_TO_POSE.get(fiducialID).getTranslation().getDistance(robotPose.getTranslation());
    }
}