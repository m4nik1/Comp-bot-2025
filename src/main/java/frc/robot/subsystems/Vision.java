// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  PhotonPipelineResult result;
  PhotonTrackedTarget target;
  // PhotonPoseEstimator poseEstimator_reef;

  PhotonPoseEstimator[] poseEstimators = new PhotonPoseEstimator[2];

  final String[] camera_names = {
      "arducam-558",
      "high-arducam-558"
  };

  PhotonCamera[] cameras = new PhotonCamera[2];

  final Transform3d[] robotToCamTransforms = {

      new Transform3d(new Translation3d(-0.095, 0.3302, 0.6), // Pitch is positive 20 degrees check that though
          // new Transform3d(Units.inchesToMeters(10), Units.inchesToMeters(4.25),
          // Units.inchesToMeters(23),
          new Rotation3d(0, Rotation2d.fromDegrees(-20).getRadians(), Rotation2d.fromDegrees(-20).getRadians())),

      new Transform3d(new Translation3d(-0.095, 0.3302, 0.75), // Higher camera
          // new Transform3d(Units.inchesToMeters(10), Units.inchesToMeters(4.25),
          // Units.inchesToMeters(23+14),
          new Rotation3d(0, 0, 0))
  };

  private Matrix<N3, N1> curStdDevs;
  double MAX_SINGLE_ABIGUITY = 0.06; // These are for single targets only

  boolean targetVisible = false;
  double targetYaw = 0.0;
  double turn = 0.0;
  double vision_kP = 1;

  Matrix<N3, N1> kSingleTagStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 0.1, 0.1, 0.05 });
  Matrix<N3, N1> kMultiTagStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 0.025, 0.025, 0.0125 });
  Matrix<N3, N1> kStateDriveStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[] { 0.01, 0.01, 0.005 });

  AprilTagFieldLayout aprilTagFieldLayout;

  public Vision() {

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    // This takes all the tags into account for estimating pose
    // poseEstimator_reef = new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    // poseEstimator_reef.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Making multiple Pose Estimators and cameras
    for (int i = 0; i < Constants.numCameras; i++) {
      cameras[i] = new PhotonCamera(camera_names[i]);

      if (i == 0) {
        poseEstimators[i] = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
            robotToCamTransforms[i]);
      } else {
        poseEstimators[i] = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamTransforms[i]);
      }

      poseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

  }

  // Gets the robot pose on the field
  // This should be called once per loop
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPoseEstimator poseEstimator, int cameraNum) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (var tagChange : cameras[cameraNum].getAllUnreadResults()) {
      // Add std dev. in the update function
      visionEst = poseEstimator.update(tagChange); // Updates the pose estimator with camera updates
      updateEstimationStdDevs(visionEst, tagChange.targets, poseEstimator); // Calculates new std dev's
    }

    return visionEst;
  }

 

  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  public void findReefFace(int cameraNum) {
    var results = cameras[cameraNum].getAllUnreadResults();

    if (!results.isEmpty()) {
      var latestResults = results.get(results.size() - 1);
      if (latestResults.hasTargets()) {
        int tagId = latestResults.getBestTarget().getFiducialId();

        if (Constants.desiredTagIds.contains(tagId)) {
          double face = Constants.TagToFaceBlue.get(tagId);
          // double thetaFace = ((2*Math.PI)*(face/6)+Math.PI) % (2*Math.PI);

          // Transform2d facePose = Robot.reefPosesGenerate.calculateAlgaePose(thetaFace);
          SmartDashboard.putNumber("Reef Face", face);
        }
      }
    }
  }

  public List<PhotonPipelineResult> getUnreadResults(int cameraNum) {
    return cameras[cameraNum].getAllUnreadResults();
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose,
      List<PhotonTrackedTarget> targets,
      PhotonPoseEstimator poseEstimator) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      // This goes through the targets and 
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
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
        if (numTags > 1)
          estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        // if(numTags == 1 && avgDist < 1.5) {
        // estStdDevs = VecBuilder.fill(.05, .05, .025);
        // }
        if (numTags == 1 && avgDist > 3) // Checks if the distance is more than 4 meters away
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        curStdDevs = estStdDevs;
      }
    }
  }

  public void addVisionMeasurementToDriveTrain(PhotonPoseEstimator photonPoseEstimator, int cameraNum) {
    Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(photonPoseEstimator, cameraNum);

    if (!result.isPresent()) {
      return;
    }

    EstimatedRobotPose robotPose = result.get();

    boolean singleTarget = robotPose.targetsUsed.size() == 1;

    // below we are checking if we can trust the single target
    if (singleTarget) {
      PhotonTrackedTarget target = robotPose.targetsUsed.get(2);

      // This is only when single targets are detected
      if (target.getPoseAmbiguity() > MAX_SINGLE_ABIGUITY) {
        return;
      }
    }
    // Logger.recordOutput("Vision abiluity", target.getPoseAmbiguity());
    Pose2d estimatedRobotPose2d = robotPose.estimatedPose.toPose2d();

    Logger.recordOutput("Vision Pose", estimatedRobotPose2d);
    double timestampSeconds = result.get().timestampSeconds;

    RobotContainer.driveTrain.addVisionMeasurment(estimatedRobotPose2d, timestampSeconds, curStdDevs);
  }

  public void addVisionMeasurement(PhotonPoseEstimator estimator, int cameraNum) {
    List<PhotonPipelineResult> results = cameras[cameraNum].getAllUnreadResults();

     PhotonPipelineResult latestResult = results.get(results.size() - 1);
     
     Optional<EstimatedRobotPose> estimatorResult = estimator.update(latestResult);
     // Now we update the estimator and get the pose from the estimator
     Pose2d estimatedPose = estimatorResult.estimatedPose.toPose2d();

     double timestampUpdate = estimatorResult.timestampSeconds;
     List<PhotonTrackedTarget> tags = estimatorResult.targetsUsed;
     int tagCount = tags.size();

     var distanceToClosetTag = tags[0].bestCameraToTarget.translation().toTranslation().distance(new Translation2d(0, 0));

     int std_devs = 2;

     if (tagCount == 0) {
      return;
     }

     if (tagCount == 1) {
      // comparing to the distance threshold to get the right std_devs
      if (distanceToClosetTag > 2) {
        return;
      }

      if((6 <= primary_id <= 11) | (17 <= primary_id <= 11)) && (distanceToClosetTag <= 0.5) {
        std_devs = 0.25;
        if (distanceToClosetTag <= 0.75) {
          std_devs = 0.1;
        }
      }
    }

    if (tagCount >= 2) {
      std_devs = 0.7;
      if((6 <= primary_id <= 11) | (17 <= primary_id <= 11)) && (distanceToClosetTag <= 0.5) {
        std_devs = 0.5;
        if (distanceToClosetTag <= 0.25) {
          std_devs = 0.25;
        }
      }
    }

    // Now we can send the measurement to the drivetrain/SwerveEstimator
    RobotContainer.driveTrain.addVisionMeasurment(new Pose2d(estimatedPose.getX(), estimatedPose.getY(), RobotContainer.driveTrain.getYaw()), timestampUpdate, [curStdDevs, curStdDevs, 50]);

  @Override
  public void periodic() {

    for (int k = 0; k < Constants.numCameras; k++) {
      addVisionMeasurementToDriveTrain(poseEstimators[k], k);
    }
    // addVisionMeasurementToDriveTrain(poseEstimator_reef);
    // findReefFace();
  }
}
