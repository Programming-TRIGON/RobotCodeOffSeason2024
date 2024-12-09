package frc.trigon.robot.poseestimation.apriltagcamera;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public class AprilTagCameraIO {
    protected void updateInputs(AprilTagCameraInputsAutoLogged inputs) {
    }

    /**
     * Adds the simulated camera to the pose estimation simulation.
     *
     * @param robotToCamera the transform of the robot's origin point to the camera
     */
    protected void addSimulatedCameraToVisionSimulation(Transform3d robotToCamera) {
    }

    @AutoLog
    public static class AprilTagCameraInputs {
        public boolean hasResult = false;
        public double latestResultTimestampSeconds = 0;
        public Transform3d cameraSolvePNPPose = new Transform3d();
        public int[] visibleTagIDs = new int[0];
        public double bestTargetRelativeYawRadians = 0;
        public double bestTargetRelativePitchRadians = 0;
        public double distanceFromBestTag = 0;
        public double poseAmbiguity = 0;
    }
}
