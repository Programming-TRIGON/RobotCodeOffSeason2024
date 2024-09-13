package frc.trigon.robot.poseestimation.robotposesources;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public class RobotPoseSourceIO {
    protected void updateInputs(RobotPoseSourceInputsAutoLogged inputs) {
    }

    @AutoLog
    public static class RobotPoseSourceInputs {
        public boolean hasResult = false;
        public double lastResultTimestamp = 0;
        public Pose3d solvePNPPose = new Pose3d();
        public int[] visibleTagIDs = new int[0];
        public double bestTargetRelativeYaw = 0;
        public double bestTargetRelativePitch = 0;
        public double averageDistanceFromAllTags = 0;
        public double distanceFromBestTag = 0;
    }
}
