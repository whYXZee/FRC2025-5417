// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Limelight. */  
  Optional<EstimatedRobotPose> results1;
  Optional<EstimatedRobotPose> results2;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  Transform3d camera1_transform = new Transform3d(new Translation3d(0.0, 0.0254, 0.6096), new Rotation3d(0,0,Math.PI/2)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  Transform3d camera2_transform = new Transform3d(new Translation3d(0.0, 0.0254, 0.6096), new Rotation3d(0,0,-Math.PI/2)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  public Pose2d pose1 = new Pose2d();
  public double timestamp1 = 0.0;
  public double timestamp2 = 0.0;
  public Pose2d pose2 = new Pose2d();

  public Field2d field1 = new Field2d();
  public Field2d field2 = new Field2d();

  public int isdata = 0;
  // public LongSupplier datsupp = () -> isdata;

  private final PhotonCamera camera1;

  private final PhotonPoseEstimator photonEstimator1;

  public Vision() {
    camera1 = new PhotonCamera("camera1");

    photonEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1_transform);
    photonEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera camera, PhotonPoseEstimator estimator) {
    Optional<EstimatedRobotPose> estimatedPose = Optional.empty(); 
    
    if (camera.getLatestResult().hasTargets()) {
      isdata = camera.getLatestResult().getBestTarget().getFiducialId();
      estimatedPose = estimator.update(camera.getLatestResult());
    }
    
    return estimatedPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    results1 = getEstimatedGlobalPose(camera1, photonEstimator1);

    if (results1.isPresent()) {
      pose1 = results1.get().estimatedPose.toPose2d();
      timestamp1 = results1.get().timestampSeconds;
    }

    if (results2.isPresent()) {
      pose2 = results2.get().estimatedPose.toPose2d();
      timestamp2 = results2.get().timestampSeconds;
    }

    field1.setRobotPose(pose1);
    field2.setRobotPose(pose2);
    
  }
}