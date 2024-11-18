package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class AprilTagPhotonCameraIO extends AprilTagCameraIO {
    protected final PhotonCamera photonCamera;

    public AprilTagPhotonCameraIO(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
    }

    @Override
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = getLatestPipelineResult();

        inputs.hasResult = latestResult != null && latestResult.hasTargets() && !latestResult.getTargets().isEmpty();
        if (inputs.hasResult) {
            updateHasResultInputs(inputs, latestResult);
            return;
        }
        updateNoResultInputs(inputs);
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }

    private void updateHasResultInputs(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        final PhotonTrackedTarget bestTarget = getBestTarget(latestResult);
        final Rotation3d bestTargetRelativeRotation3d = getBestTargetRelativeRotation(bestTarget);

        inputs.cameraSolvePNPPose = getSolvePNPPose(latestResult, bestTarget);
        inputs.latestResultTimestampSeconds = latestResult.getTimestampSeconds();
        inputs.bestTargetRelativePitchRadians = bestTargetRelativeRotation3d.getY();
        inputs.bestTargetRelativeYawRadians = bestTargetRelativeRotation3d.getZ();
        inputs.visibleTagIDs = getVisibleTagIDs(latestResult, bestTarget);
        inputs.distanceFromBestTag = getDistanceFromBestTag(bestTarget);
        inputs.poseAmbiguity = bestTarget.getPoseAmbiguity();
    }

    private void updateNoResultInputs(AprilTagCameraInputsAutoLogged inputs) {
        inputs.cameraSolvePNPPose = new Pose3d();
        inputs.visibleTagIDs = new int[0];
        inputs.distanceFromBestTag = Double.POSITIVE_INFINITY;
    }

    /**
     * Calculates the best tag from the pipeline result based on the area that the tag takes up.
     *
     * @param result the camera's pipeline result
     * @return the best target
     */
    private PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        for (PhotonTrackedTarget target : result.getTargets()) {
            if (target.getArea() > bestTarget.area)
                bestTarget = target;
        }
        return bestTarget;
    }

    /**
     * Estimates the camera's rotation relative to the apriltag.
     *
     * @param bestTag the best apriltag visible to the camera
     * @return the estimated rotation
     */
    private Rotation3d getBestTargetRelativeRotation(PhotonTrackedTarget bestTag) {
        return bestTag.getBestCameraToTarget().getRotation();
    }

    /**
     * Estimates the camera's pose using Solve PNP using as many tags as possible.
     *
     * @param result the camera's pipeline result
     * @return the estimated pose
     */
    private Pose3d getSolvePNPPose(PhotonPipelineResult result, PhotonTrackedTarget bestTarget) {
        if (result.getMultiTagResult().isPresent()) {
            final Transform3d cameraPoseTransform = result.getMultiTagResult().get().estimatedPose.best;
            return new Pose3d().plus(cameraPoseTransform).relativeTo(FieldConstants.APRIL_TAG_FIELD_LAYOUT.getOrigin());
        }

        final Pose3d tagPose = FieldConstants.TAG_ID_TO_POSE.get(bestTarget.getFiducialId());
        final Transform3d targetToCamera = bestTarget.getBestCameraToTarget().inverse();
        return tagPose.transformBy(targetToCamera);
    }

    private int[] getVisibleTagIDs(PhotonPipelineResult result, PhotonTrackedTarget bestTarget) {
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final int[] visibleTagIDs = new int[targets.size()];
        boolean hasSeenBestTarget = false;

        visibleTagIDs[0] = bestTarget.getFiducialId();
        for (int i = 0; i < visibleTagIDs.length; i++) {
            final int targetID = targets.get(i).getFiducialId();
            if (targetID == visibleTagIDs[0]) {
                hasSeenBestTarget = true;
                continue;
            }
            visibleTagIDs[hasSeenBestTarget ? i : i + 1] = targetID;
        }
        return visibleTagIDs;
    }

    private double getDistanceFromBestTag(PhotonTrackedTarget bestTag) {
        return bestTag.getBestCameraToTarget().getTranslation().getNorm();
    }
}
