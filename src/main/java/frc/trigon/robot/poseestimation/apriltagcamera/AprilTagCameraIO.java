package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public class AprilTagCameraIO {
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class RobotPoseSourceInputs {
        public boolean hasResult = false;
        public double latestResultTimestampSeconds = 0;
        public Pose3d cameraSolvePNPPose = new Pose3d();
        public int[] visibleTagIDs = new int[0];
        public double averageDistanceFromAllTags = 0;
        public double distanceFromBestTag = 0;
    }
}