package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public class AprilTagCameraIO {
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
    }

    protected void addSimCamera(Transform3d robotToCamera) {
    }

    @AutoLog
    public static class AprilTagCameraInputs {
        public boolean hasResult = false;
        public double latestResultTimestampSeconds = 0;
        public Pose3d cameraSolvePNPPose = new Pose3d();
        public int[] visibleTagIDs = new int[0];
        public double bestTargetRelativeYawRadians = 0;
        public double bestTargetRelativePitchRadians = 0;
        public double distanceFromBestTag = 0;
    }
}
