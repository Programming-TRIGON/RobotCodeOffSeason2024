package frc.trigon.robot.poseestimation.robotposesources;

import org.trigon.utilities.LimelightHelpers;

public class AprilTagLimelightIO extends RobotPoseSourceIO {
    private final String hostname;

    protected AprilTagLimelightIO(String hostname) {
        this.hostname = hostname;
    }

    @Override
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
        inputs.hasResult = LimelightHelpers.getTV(hostname);
        if (inputs.hasResult) {
            final LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(hostname);
            inputs.cameraPose = RobotPoseSource.pose3dToDoubleArray(results.getBotPose3d_wpiBlue());
            inputs.lastResultTimestamp = results.timestamp_LIMELIGHT_publish;
        }
    }
}
