// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final Field2d visionField = new Field2d();

  private CommandSwerveDrivetrain m_drivetrain = null;

  PhotonCamera frontCamera = new PhotonCamera("FrontCamera");
   private PhotonPipelineResult frontResult = null;
   private PhotonTrackedTarget frontTrackedTarget = null;
   private PhotonPoseEstimator frontPoseEstimator = new PhotonPoseEstimator(
     AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
     PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
     VisionConstants.FRONT_CAMERA_POSITION
   );
   private EstimatedRobotPose frontEstimatedRobotPose = null;

  //    PhotonCamera sideCamera = new PhotonCamera("Side Camera");
  //  private PhotonPipelineResult sideResult = null;
  //  private PhotonTrackedTarget sideTrackedTarget = null;
  //  private PhotonPoseEstimator sidePoseEstimator = new PhotonPoseEstimator(
  //    AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
  //    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
  //    VisionConstants.SIDE_CAMERA_POSITION
  //  );
  //  private EstimatedRobotPose sideEstimatedRobotPose = null;

   private Matrix<N3, N1> curStdDevs;
   private final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1.5, 1.5, 6); //1.5, 1.5, 6  VecBuilder.fill(4, 4, 8); (2 , 2 , 8)
   private final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, 0.75); //0.3, 0.3, .75   0.5,0.5,1
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;

    //SmartDashboard.putData("Vision Field", visionField);
    
    //CameraServer.startAutomaticCapture();
  }

  public void addVisionPose2d(Pose2d pose2d, double timestampSeconds) {
     //SmartDashboard.putNumber("aVP2d pose2d X", pose2d.getX());
     //SmartDashboard.putNumber("aVP2d pose2d Y", pose2d.getY());
     //SmartDashboard.putNumber("aVP2d pose2d Rot", pose2d.getRotation().getDegrees());
     //SmartDashboard.putNumber("aVP2d timestampSeconds", timestampSeconds);
     // Sets trust value for vision measurements
     // charizardsSkateboard.setVisionMeasurementStdDevs(curStdDevs);
     // charizardsSkateboard.addVisionMeasurement(pose2d, timestampSeconds);
     m_drivetrain.setVisionMeasurementStdDevs(kSingleTagStdDevs);
     /*
     double xUpperLimitOfTrustBox = m_drivetrain.getState().Pose.getX() + RadiusOfToleranceSquare;
     double xLowerLimitOfTrustBox = m_drivetrain.getState().Pose.getX() - RadiusOfToleranceSquare;
     double yUpperLimitOfTrustBox = m_drivetrain.getState().Pose.getY() + RadiusOfToleranceSquare;
     double yLowerLimitOfTrustBox = m_drivetrain.getState().Pose.getY() - RadiusOfToleranceSquare;
     
  
     //  SmartDashboard.putNumber("charizardsSkateboard X", charizardsSkateboard.getState().Pose.getX());
     //  SmartDashboard.putNumber("charizardsSkateboard Y", charizardsSkateboard.getState().Pose.getY());
     //  SmartDashboard.putNumber("charizardsSkateboard Rot", charizardsSkateboard.getState().Pose.getRotation().getDegrees());
     if (MathUtil.isNear(0, m_drivetrain.getState().Speeds.vxMetersPerSecond, 0.1) && MathUtil.isNear(0, m_drivetrain.getState().Speeds.vyMetersPerSecond, 0.1)) {
       //are we moving,, if so then add trust box  
       m_drivetrain.addVisionMeasurement(pose2d, timestampSeconds);
     } else if(pose2d.getX() >= xLowerLimitOfTrustBox && pose2d.getX() <= xUpperLimitOfTrustBox && pose2d.getY() >= yLowerLimitOfTrustBox && pose2d.getY() <= yUpperLimitOfTrustBox) {
       m_drivetrain.addVisionMeasurement(pose2d, timestampSeconds);
     } */

     m_drivetrain.addVisionMeasurement(pose2d, timestampSeconds);
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get camera results
     frontResult = frontCamera.getLatestResult();
     // backLeftResult = backLeftCamera.getLatestResult();
     // backRightResult = backRightCamera.getLatestResult(); //NEEDS CHANGING BEFORE WE RETIRE FOR MICHALS SAFTEY NEXT YEAR

     // Camera Pointing Left
     // Try to update "latestRobotPose" with a new "EstimatedRobotPose" using a "PhotonPoseEstimator"
     // If "latestRobotPose" is updated, call addVisionPose2d() and pass the updated "latestRobotPose" as an argument
     try {
       // Only accepts camera results if they see more than 1 april tag, or if it sees 1 april tag and the poseAmbiguity is low
       // COMMENT OUT THE LINE BELOW THIS AND IT'S CLOSING BRACKETS IF THIS DOESN'T WORK
       if ((frontResult.getTargets().size() == 1 && frontResult.getBestTarget().poseAmbiguity < VisionConstants.AMBIGUITY_RATIO_CUTOFF) 
       || frontResult.getTargets().size() > 1) {
         frontEstimatedRobotPose = frontPoseEstimator.update(frontResult).get();
         //updateEstimationStdDevs(leftPoseEstimator.update(leftResult), cameraLeft.getAllUnreadResults().get(0).getTargets());
         addVisionPose2d(frontEstimatedRobotPose.estimatedPose.toPose2d(), frontEstimatedRobotPose.timestampSeconds);
         visionField.setRobotPose(frontEstimatedRobotPose.estimatedPose.toPose2d());
       }
     } catch (Exception e) {
      
       frontEstimatedRobotPose = null;
     }

  //   // Same thing but for the side camera
  //     sideResult = sideCamera.getLatestResult();

  //     try {
  //      // Only accepts camera results if they see more than 1 april tag, or if it sees 1 april tag and the poseAmbiguity is low
  //      // COMMENT OUT THE LINE BELOW THIS AND IT'S CLOSING BRACKETS IF THIS DOESN'T WORK
  //      if ((sideResult.getTargets().size() == 1 && sideResult.getBestTarget().poseAmbiguity < VisionConstants.AMBIGUITY_RATIO_CUTOFF) 
  //      || sideResult.getTargets().size() > 1) {
  //        sideEstimatedRobotPose = sidePoseEstimator.update(sideResult).get();
  //        //updateEstimationStdDevs(leftPoseEstimator.update(leftResult), cameraLeft.getAllUnreadResults().get(0).getTargets());
  //        addVisionPose2d(sideEstimatedRobotPose.estimatedPose.toPose2d(), sideEstimatedRobotPose.timestampSeconds);
  //        visionField.setRobotPose(sideEstimatedRobotPose.estimatedPose.toPose2d());
  //      }
  //    } catch (Exception e) {
  //      sideEstimatedRobotPose = null;
  //    } 

  //    try {
  //     visionField.setRobotPose(frontEstimatedRobotPose.estimatedPose.toPose2d());
  //    } catch(Exception e) {
        
  //    }
     
   }
}

