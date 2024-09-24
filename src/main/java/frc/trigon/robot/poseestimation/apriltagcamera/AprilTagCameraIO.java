package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class AprilTagCameraIO {
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class RobotPoseSourceInputs {
        public boolean hasResult = false;
        public double latestResultTimestampSeconds = 0;
        public Rotation2d solvePNPHeading = new Rotation2d();
        public int[] visibleTagIDs = new int[0];
        public double bestTargetRelativeYawRadians = 0;
        public double bestTargetRelativePitchRadians = 0;
        public double averageDistanceFromAllTags = 0;
        public double distanceFromBestTag = 0;
    }
}
