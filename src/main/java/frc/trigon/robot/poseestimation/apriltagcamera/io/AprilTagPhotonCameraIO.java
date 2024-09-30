package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.RobotPoseSourceInputsAutoLogged;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagPhotonCameraIO extends AprilTagCameraIO {
    private final PhotonCamera photonCamera;

    public AprilTagPhotonCameraIO(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = photonCamera.getLatestResult();

            inputs.hasResult = latestResult.hasTargets() && !latestResult.getTargets().isEmpty();
            if (inputs.hasResult)
                updateHasResultInputs(inputs, latestResult);
            else
                updateNoResultInputs(inputs);
    }

    private void updateHasResultInputs(RobotPoseSourceInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        inputs.cameraSolvePNPPose = getSolvePNPPose(latestResult);
        inputs.latestResultTimestampSeconds = latestResult.getTimestampSeconds();
        inputs.visibleTagIDs = getVisibleTagIDs(latestResult);
        inputs.averageDistanceFromAllTags = getAverageDistanceFromAllTags(latestResult);
        inputs.distanceFromBestTag = getDistanceFromBestTag(latestResult);
    }

    private void updateNoResultInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.visibleTagIDs = new int[]{};
        inputs.cameraSolvePNPPose = new Pose3d();
    }

    /**
     * Estimates the camera's pose using Solve PNP using as many tags as possible.
     *
     * @param result the camera's pipeline result
     * @return the estimated pose
     */
    private Pose3d getSolvePNPPose(PhotonPipelineResult result) {
        final PNPResult multitagPose = result.getMultiTagResult().estimatedPose;
        if (multitagPose.isPresent && multitagPose.ambiguity < AprilTagCameraConstants.MAXIMUM_AMBIGUITY) {
            final Transform3d cameraPoseTransform = multitagPose.best;
            return new Pose3d().plus(cameraPoseTransform).relativeTo(FieldConstants.APRIL_TAG_FIELD_LAYOUT.getOrigin());
        }

        final PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget.getPoseAmbiguity() > AprilTagCameraConstants.MAXIMUM_AMBIGUITY)
            return new Pose3d();

        final Pose3d tagPose = FieldConstants.TAG_ID_TO_POSE.get(bestTarget.getFiducialId());
        final Transform3d targetToCamera = bestTarget.getBestCameraToTarget().inverse();
        return tagPose
                .transformBy(targetToCamera)
                .transformBy(AprilTagCameraConstants.TAG_OFFSET);
    }

    private int[] getVisibleTagIDs(PhotonPipelineResult result) {
        final int[] visibleTagIDs = new int[result.getTargets().size()];
        visibleTagIDs[0] = result.getBestTarget().getFiducialId();
        int idAddition = 1;

        for (int i = 0; i < visibleTagIDs.length; i++) {
            final int currentID = result.getTargets().get(i).getFiducialId();

            if (currentID == visibleTagIDs[0]) {
                idAddition = 0;
                continue;
            }
            visibleTagIDs[i + idAddition] = currentID;
        }

        return visibleTagIDs;
    }

    private double getAverageDistanceFromAllTags(PhotonPipelineResult result) {
        final int tagsSeen = result.getTargets().size();
        double totalTagDistance = 0;

        for (int i = 0; i < tagsSeen; i++)
            totalTagDistance += result.getTargets().get(i).getBestCameraToTarget().getTranslation().getNorm();

        return totalTagDistance / tagsSeen;
    }

    private double getDistanceFromBestTag(PhotonPipelineResult result) {
        return result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
    }
}
