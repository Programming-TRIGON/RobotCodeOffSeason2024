package frc.trigon.robot.misc;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.constants.CameraConstants;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.trigon.utilities.mirrorable.MirrorableTranslation3d;

import java.util.List;

public class RobotShowcase {
    private final PhotonCamera rearTagCamera = new PhotonCamera("RearTagCamera");

    public void periodic() {
//        System.out.println("RobotShowcase periodic has targets " + getLatestPipelineResult().hasTargets());
//        if (!getLatestPipelineResult().hasTargets()) {
//            Logger.recordOutput("Cameras/RobotShowcase/HasTargets", false);
//            return;
//        }
        Logger.recordOutput("Cameras/RobotShowcase/HasTargets", getLatestPipelineResult().hasTargets());
        Logger.recordOutput("Cameras/RobotShowcase/BestTargetID", getBestResult() == null ? -1 : getBestResult().getBestTarget().getFiducialId());
        Logger.recordOutput("Cameras/RobotShowcase/VisibleTagIDs", getVisibleTagIDs(getBestResult()));
        Logger.recordOutput("Cameras/RobotShowcase/BestTagTranslationRelativeToRobot", getBestTagTranslationRelativeToRobot().get());
    }

    public MirrorableTranslation3d getBestTagTranslationRelativeToRobot() {
        final PhotonPipelineResult bestResult = getBestResult();
        if (bestResult == null)
            return new MirrorableTranslation3d(new Translation3d(), false);
        final Transform3d cameraToTagTransform = getBestTagTranslation(bestResult);
        return new MirrorableTranslation3d(CameraConstants.REAR_CENTER_TO_CAMERA_POSE.transformBy(cameraToTagTransform).getTranslation(), false);
    }

    private Transform3d getBestTagTranslation(PhotonPipelineResult result) {
        final PhotonTrackedTarget bestTarget = result.getBestTarget();
        return bestTarget.getBestCameraToTarget();
    }

    private PhotonPipelineResult getBestResult() {
        final PhotonPipelineResult rearResult = getLatestPipelineResult();
        if (!rearResult.hasTargets())
            return null;
        return rearResult;
    }

    private PhotonPipelineResult getLatestPipelineResult() {
        final List<PhotonPipelineResult> unreadResults = rearTagCamera.getAllUnreadResults();
//        System.out.println("RobotShowcase camera unreadResults is empty: " + unreadResults.isEmpty());
        return unreadResults.isEmpty() ? new PhotonPipelineResult() : unreadResults.get(unreadResults.size() - 1);
    }

    private int[] getVisibleTagIDs(PhotonPipelineResult result) {
        if (result == null)
            return new int[0];
        final PhotonTrackedTarget bestTarget = result.getBestTarget();
        final List<PhotonTrackedTarget> targets = result.getTargets();
        final int[] visibleTagIDs = new int[targets.size()];
        visibleTagIDs[0] = bestTarget.getFiducialId();

        boolean hasSeenBestTarget = false;
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
}
