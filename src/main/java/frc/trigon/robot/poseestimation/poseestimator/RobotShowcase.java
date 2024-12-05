package frc.trigon.robot.poseestimation.poseestimator;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.constants.CameraConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.trigon.utilities.mirrorable.MirrorableTranslation3d;

import java.util.List;

public class RobotShowcase {
    private final PhotonCamera
            frontTagCamera = new PhotonCamera("FrontTagCamera"),
            rearTagCamera = new PhotonCamera("RearTagCamera");
    private boolean isGettingResultFromFrontCamera = true;

    public MirrorableTranslation3d getBestTagTranslationRelativeToRobot() {
        final PhotonPipelineResult bestResult = getBestResult();
        if (bestResult == null)
            return new MirrorableTranslation3d(new Translation3d(), false);
        final Transform3d cameraToTagTransform = getBestTagTranslation(bestResult);

        if (isGettingResultFromFrontCamera)
            return new MirrorableTranslation3d(CameraConstants.FRONT_CENTER_TO_CAMERA_POSE.transformBy(cameraToTagTransform).getTranslation(), false);
        return new MirrorableTranslation3d(CameraConstants.REAR_CENTER_TO_CAMERA_POSE.transformBy(cameraToTagTransform).getTranslation(), false);
    }

    private Transform3d getBestTagTranslation(PhotonPipelineResult result) {
        final PhotonTrackedTarget bestTarget = result.getBestTarget();
        return bestTarget.getBestCameraToTarget();
    }

    private PhotonPipelineResult getBestResult() {
        final PhotonPipelineResult frontResult = getLatestPipelineResult(frontTagCamera);
        final PhotonPipelineResult rearResult = getLatestPipelineResult(rearTagCamera);

        if (!frontResult.hasTargets()) {
            if (!rearResult.hasTargets())
                return null;
            isGettingResultFromFrontCamera = false;
            return rearResult;
        }
        if (!rearResult.hasTargets()) {
            if (!frontResult.hasTargets())
                return null;
            isGettingResultFromFrontCamera = true;
            return frontResult;
        }
        if (frontResult.getBestTarget().getArea() > rearResult.getBestTarget().getArea()) {
            isGettingResultFromFrontCamera = true;
            return frontResult;
        }
        isGettingResultFromFrontCamera = false;
        return rearResult;
    }

    private PhotonPipelineResult getLatestPipelineResult(PhotonCamera camera) {
        final List<PhotonPipelineResult> unreadResults = camera.getAllUnreadResults();
        return unreadResults.isEmpty() ? new PhotonPipelineResult() : unreadResults.get(unreadResults.size() - 1);
    }
}
