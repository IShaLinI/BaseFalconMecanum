// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class Vision{
    public PhotonCamera photonCamera;
    public RobotPoseEstimator robotPoseEstimator;

    public Vision() {

        // Set up a field layout with the correct april tag locations
        AprilTagFieldLayout atfl = new AprilTagFieldLayout(FieldConstants.aprilTags, FieldConstants.fieldLength, FieldConstants.fieldWidth);

        // Forward Camera
        photonCamera = new PhotonCamera(VisionConstants.FORWARD_CAMERA_NAME);

        // ... Add other cameras here

        // Assemble the list of cameras & mount locations
        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.robotToCam));

        robotPoseEstimator = new RobotPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent() && RobotBase.isReal()) {
            return new Pair<Pose2d, Double>(
                result.get().getFirst().toPose2d(), currentTime - result.get().getSecond()
            );
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
}
