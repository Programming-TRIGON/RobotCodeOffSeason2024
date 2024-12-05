package frc.trigon.robot.poseestimation.poseestimator;

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
        final Translation3d bestTagTranslation = getBestTagTranslation(bestResult);

        if (isGettingResultFromFrontCamera)
            return new MirrorableTranslation3d(bestTagTranslation.minus(CameraConstants.FRONT_CENTER_TO_CAMERA.getTranslation()), false);
        return new MirrorableTranslation3d(bestTagTranslation.minus(CameraConstants.REAR_CENTER_TO_CAMERA.getTranslation()), false);
    }

    private Translation3d getBestTagTranslation(PhotonPipelineResult result) {
        final PhotonTrackedTarget bestTarget = result.getBestTarget();
        final Translation3d tagTranslation = bestTarget.getBestCameraToTarget().getTranslation();
        return tagTranslation;
    }

    private PhotonPipelineResult getBestResult() {
        final PhotonPipelineResult frontResult = getLatestPipelineResult(frontTagCamera);
        final PhotonPipelineResult rearResult = getLatestPipelineResult(rearTagCamera);

        if (frontResult == null && rearResult != null) {
            isGettingResultFromFrontCamera = false;
            return rearResult;
        }
        if (rearResult == null) {
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
        return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
    }
}
