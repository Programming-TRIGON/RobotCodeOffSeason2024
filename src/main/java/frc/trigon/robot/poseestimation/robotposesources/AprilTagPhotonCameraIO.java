package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.trigon.robot.constants.FieldConstants;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import java.util.List;

public class AprilTagPhotonCameraIO extends RobotPoseSourceIO {
    private final PhotonCamera photonCamera;

    protected AprilTagPhotonCameraIO(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = photonCamera.getLatestResult();

        inputs.hasResult = latestResult.hasTargets();
        if (inputs.hasResult)
            updateHasResultInputs(inputs, latestResult);
        else
            updateNoResultInputs(inputs);
    }

    /**
     * Estimates the camera's rotation relative to the apriltag.
     *
     * @param result the camera's pipeline result
     * @return the estimated rotation
     */
    private Rotation3d getBestTargetRelativeRotation(PhotonPipelineResult result) {
        final List<TargetCorner> tagCorners = result.getBestTarget().getDetectedCorners();
        final Point tagCenter = getTagCenter(tagCorners);
        if (photonCamera.getCameraMatrix().isPresent())
            return correctPixelRot(tagCenter, photonCamera.getCameraMatrix().get());
        return null;
    }

    /**
     * Estimates the camera's pose using Solve PNP using as many tags as possible.
     *
     * @param result the camera's pipeline result
     * @return the estimated pose
     */
    private Pose3d getSolvePNPPose(PhotonPipelineResult result) {
        if (result.getMultiTagResult().estimatedPose.isPresent) {
            final Transform3d cameraPoseTransform = result.getMultiTagResult().estimatedPose.best;
            return new Pose3d().plus(cameraPoseTransform).relativeTo(FieldConstants.APRIL_TAG_FIELD_LAYOUT.getOrigin());
        }

        final Transform3d targetToCamera = result.getBestTarget().getBestCameraToTarget().inverse();
        return FieldConstants.TAG_ID_TO_POSE.get(result.getBestTarget().getFiducialId()).transformBy(targetToCamera);
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

    private double getAverageDistanceFromBestTag(PhotonPipelineResult result) {
        return result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
    }

    private void updateHasResultInputs(RobotPoseSourceInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        final Rotation3d bestTargetRelativeRotation3d = getBestTargetRelativeRotation(latestResult);

        inputs.solvePNPPose = getSolvePNPPose(latestResult);
        inputs.lastResultTimestamp = latestResult.getTimestampSeconds();
        inputs.bestTargetRelativePitch = bestTargetRelativeRotation3d.getY();
        inputs.bestTargetRelativeYaw = bestTargetRelativeRotation3d.getZ();
        inputs.visibleTagIDs = getVisibleTagIDs(latestResult);
        inputs.averageDistanceFromAllTags = getAverageDistanceFromAllTags(latestResult);
        inputs.averageDistanceFromBestTag = getAverageDistanceFromBestTag(latestResult);
    }

    private void updateNoResultInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.visibleTagIDs = new int[0];
        inputs.solvePNPPose = new Pose3d();
    }

    private static Point getTagCenter(List<TargetCorner> tagCorners) {
        double tagCornerSumX = 0;
        double tagCornerSumY = 0;
        for (TargetCorner tagCorner : tagCorners) {
            tagCornerSumX += tagCorner.x;
            tagCornerSumY += tagCorner.y;
        }
        return new Point(tagCornerSumX / tagCorners.size(), tagCornerSumY / tagCorners.size());
    }

    private static Rotation3d correctPixelRot(Point pixel, Matrix<N3, N3> camIntrinsics) {
        double fx = camIntrinsics.get(0, 0);
        double cx = camIntrinsics.get(0, 2);
        double xOffset = cx - pixel.x;

        double fy = camIntrinsics.get(1, 1);
        double cy = camIntrinsics.get(1, 2);
        double yOffset = cy - pixel.y;

        // calculate yaw normally
        var yaw = new Rotation2d(fx, xOffset);
        // correct pitch based on yaw
        var pitch = new Rotation2d(fy / Math.cos(Math.atan(xOffset / fx)), -yOffset);

        return new Rotation3d(0, pitch.getRadians(), yaw.getRadians());
    }
}
