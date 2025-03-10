// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.StackWalker.Option;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  PhotonCamera camera;
  PhotonPipelineResult result;
  PhotonTrackedTarget target;
  PhotonPoseEstimator poseEstimator;
  private Matrix<N3, N1> curStdDevs;
  private Matrix<N3, N1> kSingleTagStdDevs;
  private Matrix<N3, N1> kMultiTagStdDevs;



  Transform3d robotToCam;

  boolean targetVisible = false;
  double targetYaw = 0.0;
  double turn = 0.0;
  double vision_kP = 1;

  AprilTagFieldLayout aprilTagFieldLayout;
  public Vision() {

    camera = new PhotonCamera("Arducam_OV9281");
    
    kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    // result = camera.getLatestResult();
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    // target = result.getBestTarget();
    
    // TODO:
    // Camera position from the center of the Robot
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 

    // This takes all the tags into account for estimating pose
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  // Gets the robot pose on the field
  // This should be called once per loop
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for(var tagChange : camera.getAllUnreadResults()) {
      // Add std dev. in the update function
      visionEst = poseEstimator.update(tagChange); // Updates the pose estimator with camera updates
      updateEstimationStdDevs(visionEst, tagChange.targets);
    }

    return visionEst;
  }

  public List<PhotonPipelineResult> getUnreadResults() {
    return camera.getAllUnreadResults();
  }

  private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }



  @Override
  public void periodic() {
    
    // getEstimatedGlobalPose(); // This updates the pose of the robot
    SmartDashboard.putNumber("turn vision", turn);
    SmartDashboard.putNumber("Vision Yaw ", targetYaw);

    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Target found", result.hasTargets());
    // SmartDashboard.putNumber("Pose found", target.getPoseAmbiguity());
    // SmartDashboard.putNumber("Camera Yaw", target.getYaw());
    // SmartDashboard.putNumber("Camera Pitch", target.getPitch());

  }
}
