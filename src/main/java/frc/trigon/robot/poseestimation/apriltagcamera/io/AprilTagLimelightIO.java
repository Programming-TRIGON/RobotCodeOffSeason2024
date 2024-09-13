package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
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
        inputs.lastResultTimestampSeconds = Units.millisecondsToSeconds(results.timestamp_LIMELIGHT_publish);
        inputs.visibleTagIDs = getVisibleTagIDs(results);
        inputs.bestTargetRelativeYawRadians = getBestTargetRelativeRotation(results).getZ();
        inputs.bestTargetRelativePitchRadians = getBestTargetRelativeRotation(results).getY();
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

    private Rotation3d getBestTargetRelativeRotation(LimelightHelpers.Results results) {
        final LimelightHelpers.LimelightTarget_Fiducial targetTag = results.targets_Fiducials[0];
        return new Rotation3d(0, targetTag.tx, targetTag.ty);
    }

    private double getAverageDistanceFromAllTags(LimelightHelpers.Results results) {
        final LimelightHelpers.LimelightTarget_Fiducial[] targetFiducials = results.targets_Fiducials;

        double totalDistanceFromTags = 0;
        for (LimelightHelpers.LimelightTarget_Fiducial targetFiducial : targetFiducials)
            totalDistanceFromTags += getDistanceFromTag((int) targetFiducial.fiducialID, results.getBotPose3d_wpiBlue());
        return totalDistanceFromTags /= results.targets_Fiducials.length;
    }

    private double getDistanceFromBestTag(LimelightHelpers.Results results) {
        return getDistanceFromTag((int) results.targets_Fiducials[0].fiducialID, results.getBotPose3d_wpiBlue());
    }

    private double getDistanceFromTag(int fiducialID, Pose3d estimatedRobotPose) {
        return FieldConstants.TAG_ID_TO_POSE.get(fiducialID).getTranslation().getDistance(estimatedRobotPose.getTranslation());
    }
}