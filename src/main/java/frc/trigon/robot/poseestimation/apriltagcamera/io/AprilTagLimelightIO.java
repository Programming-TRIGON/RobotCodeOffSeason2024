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
        final LimelightHelpers.Results results = LimelightHelpers.getLatestResults(hostname).targetingResults;

        if (results.targets_Fiducials.length > 0)
            inputs.hasResult = LimelightHelpers.getTV(hostname);
        else
            inputs.hasResult = false;

        if (inputs.hasResult)
            updateHasResultInputs(inputs);
        else
            updateNoResultInputs(inputs);
    }

    private void updateHasResultInputs(RobotPoseSourceInputsAutoLogged inputs) {
        final LimelightHelpers.Results results = LimelightHelpers.getLatestResults(hostname).targetingResults;
        final Rotation3d bestTagRelativeRotation = getBestTargetRelativeRotation(results);

        inputs.solvePNPPose = results.getBotPose3d_wpiBlue();
        inputs.latestResultTimestampSeconds = results.timestamp_RIOFPGA_capture;
        inputs.visibleTagIDs = getVisibleTagIDs(results);
        inputs.bestTargetRelativeYawRadians = bestTagRelativeRotation.getZ();
        inputs.bestTargetRelativePitchRadians = bestTagRelativeRotation.getY();
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

    /**
     * Estimates the camera's rotation relative to the apriltag.
     *
     * @param results the camera's pipeline result
     * @return the estimated rotation
     */
    private Rotation3d getBestTargetRelativeRotation(LimelightHelpers.Results results) {
        final LimelightHelpers.LimelightTarget_Fiducial bestTag = getBestTarget(results);
        return new Rotation3d(0, Units.degreesToRadians(bestTag.tx), Units.degreesToRadians(bestTag.ty));
    }

    private double getAverageDistanceFromAllTags(LimelightHelpers.Results results) {
        final LimelightHelpers.LimelightTarget_Fiducial[] targetFiducials = results.targets_Fiducials;

        double totalDistanceFromTags = 0;
        for (LimelightHelpers.LimelightTarget_Fiducial targetFiducial : targetFiducials)
            totalDistanceFromTags += getDistanceFromTag((int) targetFiducial.fiducialID, results.getBotPose3d_wpiBlue());
        return totalDistanceFromTags / results.targets_Fiducials.length;
    }

    private double getDistanceFromBestTag(LimelightHelpers.Results results) {
        return getDistanceFromTag((int) getBestTarget(results).fiducialID, results.getBotPose3d_wpiBlue());
    }

    private double getDistanceFromTag(int fiducialID, Pose3d estimatedRobotPose) {
        return FieldConstants.TAG_ID_TO_POSE.get(fiducialID).getTranslation().getDistance(estimatedRobotPose.getTranslation());
    }

    private LimelightHelpers.LimelightTarget_Fiducial getBestTarget(LimelightHelpers.Results results) {
        return results.targets_Fiducials[0];
    }
}