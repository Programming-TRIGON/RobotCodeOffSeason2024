package frc.trigon.robot.poseestimation.apriltagcamera.io;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import frc.trigon.robot.constants.FieldConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraInputsAutoLogged;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import java.util.List;

public class AprilTagPhotonCameraIO extends AprilTagCameraIO {
    private final PhotonCamera photonCamera;

    public AprilTagPhotonCameraIO(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
    }

    @Override
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = photonCamera.getLatestResult();

        inputs.hasResult = latestResult.hasTargets() && !latestResult.getTargets().isEmpty();
        if (inputs.hasResult)
            updateHasResultInputs(inputs, latestResult);
        else
            updateNoResultInputs(inputs);
    }

    private void updateHasResultInputs(AprilTagCameraInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        final Rotation3d bestTargetRelativeRotation3d = getBestTargetRelativeRotation(latestResult);

        inputs.cameraSolvePNPPose = getSolvePNPPose(latestResult);
        inputs.latestResultTimestampSeconds = latestResult.getTimestampSeconds();
        inputs.bestTargetRelativePitchRadians = bestTargetRelativeRotation3d.getY();
        inputs.bestTargetRelativeYawRadians = bestTargetRelativeRotation3d.getZ();
        inputs.visibleTagIDs = getVisibleTagIDs(latestResult);
        inputs.distanceFromBestTag = getDistanceFromBestTag(latestResult);
    }

    private void updateNoResultInputs(AprilTagCameraInputsAutoLogged inputs) {
        inputs.visibleTagIDs = new int[]{};
        inputs.cameraSolvePNPPose = new Pose3d();
    }

    private Point getTagCenter(List<TargetCorner> tagCorners) {
        double tagCornerSumX = 0;
        double tagCornerSumY = 0;
        for (TargetCorner tagCorner : tagCorners) {
            tagCornerSumX += tagCorner.x;
            tagCornerSumY += tagCorner.y;
        }
        return new Point(tagCornerSumX / tagCorners.size(), tagCornerSumY / tagCorners.size());
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

        final Pose3d rawTagPose = FieldConstants.TAG_ID_TO_POSE.get(result.getBestTarget().getFiducialId());
        final Pose3d tagPose = rawTagPose.transformBy(AprilTagCameraConstants.TAG_OFFSET);
        final Transform3d targetToCamera = result.getBestTarget().getBestCameraToTarget().inverse();
        return tagPose.transformBy(targetToCamera);
    }

    private int[] getVisibleTagIDs(PhotonPipelineResult result) {
        final int[] visibleTagIDs = new int[result.getTargets().size()];
        
        for (int i = 1; i < visibleTagIDs.length; i++)
            visibleTagIDs[i] = result.getTargets().get(i).getFiducialId();
        return visibleTagIDs;
    }

    private double getDistanceFromBestTag(PhotonPipelineResult result) {
        return result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
    }

    private Rotation3d correctPixelRot(Point pixel, Matrix<N3, N3> camIntrinsics) {
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
