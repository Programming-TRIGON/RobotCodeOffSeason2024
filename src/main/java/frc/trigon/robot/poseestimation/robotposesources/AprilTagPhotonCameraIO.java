package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
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

//    private void logVisibleTags(boolean hasResult, PhotonPipelineResult result) {
//        if (!hasResult) {
//            Logger.recordOutput("VisibleTags/" + photonCamera.getName(), new Pose2d[0]);
//            return;
//        }
//
//        final Pose2d[] visibleTagPoses = new Pose2d[result.getTargets().size()];
//        for (int i = 0; i < visibleTagPoses.length; i++) {
//            final int currentId = result.getTargets().get(i).getFiducialId();
//            final Pose2d currentPose = RobotPoseSourceConstants.TAG_ID_TO_POSE.get(currentId).toPose2d();
//            visibleTagPoses[i] = currentPose;
//        }
//        Logger.recordOutput("VisibleTags/" + photonCamera.getName(), visibleTagPoses);
//    }
//
//    private double getAverageDistanceFromTags(PhotonPipelineResult result) {
//        final List<PhotonTrackedTarget> targets = result.targets;
//        double distanceSum = 0;
//
//        for (PhotonTrackedTarget currentTarget : targets) {
//            final Translation2d distanceTranslation = currentTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
//            distanceSum += distanceTranslation.getNorm();
//        }
//
//        return distanceSum / targets.size();
//    }

    private Pose3d getSolvePNPPose(PhotonPipelineResult result) {
        Transform3d estimatedPose = result.getBestTarget().getBestCameraToTarget();
        if (result.getMultiTagResult().estimatedPose.isPresent) {
            estimatedPose = result.getMultiTagResult().estimatedPose.best;
        }
        return new Pose3d(estimatedPose.getTranslation(), estimatedPose.getRotation());
    }

    private int[] getVisibleTagIDs(PhotonPipelineResult result) {
        int[] visibleTagIDs = new int[result.getTargets().size()];
        for (int i = 0; i < visibleTagIDs.length; i++) {
            visibleTagIDs[i] = result.getTargets().get(i).getFiducialId();
        }
        return visibleTagIDs;
    }

    private void updateHasResultInputs(RobotPoseSourceInputsAutoLogged inputs, PhotonPipelineResult latestResult) {
        final Rotation3d bestTargetRelativeRotation3d = getBestTargetRelativeRotation(latestResult);
        inputs.solvePNPPose = getSolvePNPPose(latestResult);
        inputs.lastResultTimestamp = latestResult.getTimestampSeconds();
        inputs.bestTargetRelativePitch = bestTargetRelativeRotation3d.getX();
        inputs.bestTargetRelativeYaw = bestTargetRelativeRotation3d.getZ();
        inputs.visibleTagIDs = getVisibleTagIDs(latestResult);
    }

    private void updateNoResultInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.visibleTagIDs = new int[0];
        inputs.solvePNPPose = new Pose3d();
    }

    private Rotation3d getBestTargetRelativeRotation(PhotonPipelineResult result) {
        List<TargetCorner> targetCorners = result.getBestTarget().getDetectedCorners();
        double sumX = 0.0;
        double sumY = 0.0;
        for (TargetCorner t : targetCorners) {
            sumX += t.x;
            sumY += t.y;
        }

        Point tagCenter = new Point(sumX / 4, sumY / 4);

        return correctPixelRot(tagCenter, photonCamera.getCameraMatrix().get());
    }

    public static Rotation3d correctPixelRot(Point pixel, Matrix<N3, N3> camIntrinsics) {
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
