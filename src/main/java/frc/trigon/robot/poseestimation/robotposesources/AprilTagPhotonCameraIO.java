package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.List;
import java.util.Optional;

public class AprilTagPhotonCameraIO extends RobotPoseSourceIO {
    private final PhotonCamera photonCamera;

    protected AprilTagPhotonCameraIO(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        final PhotonPipelineResult latestResult = photonCamera.getLatestResult();
        final Optional<Transform3d> estimatedPose;

        if (photonCamera.getLatestResult().getMultiTagResult().estimatedPose.isPresent)
            estimatedPose = Optional.ofNullable(latestResult.getMultiTagResult().estimatedPose.best);
        else
            estimatedPose = Optional.ofNullable(latestResult.getBestTarget().getBestCameraToTarget());

        inputs.hasResult = estimatedPose.isPresent();
        if (inputs.hasResult) {
            final Transform3d estimatedRobotPose = estimatedPose.get();
            final Rotation3d bestTargetRelativeRotation3d = test(latestResult);
            inputs.cameraPose = RobotPoseSource.pose3dToDoubleArray(new Pose3d(estimatedRobotPose.getTranslation(), estimatedRobotPose.getRotation()));
            inputs.lastResultTimestamp = latestResult.getTimestampSeconds();
            inputs.bestTargetRelativePitch = bestTargetRelativeRotation3d.getX();
            inputs.bestTargetRelativeYaw = bestTargetRelativeRotation3d.getZ();
            inputs.visibleTags = new int[latestResult.getTargets().size()];
            for (int i = 0; i < inputs.visibleTags.length; i++) {
                if (i == 0) {
                    inputs.visibleTags[i] = latestResult.getBestTarget().getFiducialId();
                    continue;
                }
                final int tagId = latestResult.getTargets().get(i).getFiducialId();
                if (tagId != latestResult.getBestTarget().getFiducialId())
                    inputs.visibleTags[i] = tagId;
            }
        } else {
            inputs.visibleTags = new int[0];
            inputs.cameraPose = new double[0];
        }

        logVisibleTags(inputs.hasResult, latestResult);
    }

    private void logVisibleTags(boolean hasResult, PhotonPipelineResult result) {
        if (!hasResult) {
            Logger.recordOutput("VisibleTags/" + photonCamera.getName(), new Pose2d[0]);
            return;
        }

        final Pose2d[] visibleTagPoses = new Pose2d[result.getTargets().size()];
        for (int i = 0; i < visibleTagPoses.length; i++) {
            final int currentId = result.getTargets().get(i).getFiducialId();
            final Pose2d currentPose = RobotPoseSourceConstants.TAG_ID_TO_POSE.get(currentId).toPose2d();
            visibleTagPoses[i] = currentPose;
        }
        Logger.recordOutput("VisibleTags/" + photonCamera.getName(), visibleTagPoses);
    }

    private double getAverageDistanceFromTags(PhotonPipelineResult result) {
        final List<PhotonTrackedTarget> targets = result.targets;
        double distanceSum = 0;

        for (PhotonTrackedTarget currentTarget : targets) {
            final Translation2d distanceTranslation = currentTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
            distanceSum += distanceTranslation.getNorm();
        }

        return distanceSum / targets.size();
    }

    private Rotation3d test(PhotonPipelineResult result) {
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
