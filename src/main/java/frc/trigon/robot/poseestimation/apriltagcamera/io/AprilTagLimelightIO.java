package frc.trigon.robot.poseestimation.apriltagcamera.io;

import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraIO;
import frc.trigon.robot.poseestimation.apriltagcamera.RobotPoseSourceInputsAutoLogged;
import org.trigon.utilities.LimelightHelpers;

public class AprilTagLimelightIO extends AprilTagCameraIO {
    private final String hostname;

    public AprilTagLimelightIO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    public void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.hasResult = LimelightHelpers.getTV(hostname);
        if (inputs.hasResult) {
            final LimelightHelpers.Results results = LimelightHelpers.getLatestResults(hostname).targetingResults;
            inputs.solvePNPPose = results.getBotPose3d_wpiBlue();
            inputs.lastResultTimestamp = results.timestamp_LIMELIGHT_publish;
        }
    }
}
