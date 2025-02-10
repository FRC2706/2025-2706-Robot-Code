// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
//imports
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import frc.robot.Config.PhotonConfig;
import frc.robot.Config.PhotonConfig.PhotonPositions;
import frc.robot.commands.PhotonMoveToTarget;
import frc.robot.commands.RumbleJoystick;

//class
public class PhotonSubsystem extends SubsystemBase {

  //constants
  
  //declarations
  private static PhotonSubsystem instance;
  private DoubleArrayPublisher pubSetPoint;
  public DoublePublisher pubBestTagHeading, pubTargetRobotHeading;
  private IntegerPublisher pubBestTagId;
  private StringPublisher pub3DTagsDebugMsg;
  private PhotonCamera camera1;
  private Translation2d targetPos;
  private Rotation2d targetRotation;
  private LinearFilter filteryaw = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private LinearFilter filterX = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private LinearFilter filterY = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private int numSamples;
  private double recentTimeStamp = 0;
  private int bestTagId;
  private Rotation2d bestTagHeading;
  private Rotation2d targetRobotHeading;

  private IntegerEntry intakeCameraInputSaveImgEntry;

  private AprilTagFieldLayout aprilTagFieldLayout;
 
  public static PhotonSubsystem getInstance(){
    if (instance == null){
      SubsystemChecker.subsystemConstructed(SubsystemType.PhotonSubsystem);
      instance = new PhotonSubsystem();
    }
    return instance;
  }

  /** Creates a new photonAprilTag. */
  private PhotonSubsystem() {
    //name of camera, change if using multiple cameras
    camera1 = new PhotonCamera(PhotonConfig.apriltagCameraName);
    //networktable publishers
    NetworkTable photonTable = NetworkTableInstance.getDefault().getTable(PhotonConfig.networkTableName);
    pubBestTagId = photonTable.getIntegerTopic("BestTagId").publish(PubSubOption.periodic(0.02));
    pubBestTagHeading = photonTable.getDoubleTopic("BestTagHeading (deg)").publish(PubSubOption.periodic(0.02));
    pubTargetRobotHeading = photonTable.getDoubleTopic("TargetRobotHeading (deg)").publish(PubSubOption.periodic(0.02));
    pubSetPoint = photonTable.getDoubleArrayTopic("SetPoint").publish(PubSubOption.periodic(0.02));
    pub3DTagsDebugMsg = photonTable.getStringTopic("3DTagsDebugMsg").publish(PubSubOption.periodic(0.02));
    SmartDashboard.putData("command reset id",Commands.runOnce(()->reset()));

    reset();

    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    } catch (Exception e) {
      aprilTagFieldLayout = null;
      DriverStation.reportError("Merge's PhotonSubsystem failed to create the apriltag layout. ", false);
    }

    // Intake camera snapping photons ???
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable(PhotonConfig.frontCameraName);
    intakeCameraInputSaveImgEntry = intakeTable.getIntegerTopic("inputSaveImgCmd").getEntry(0);
  }

  public void reset() {
    filterX.reset();
    filterY.reset();
    filteryaw.reset();
    //set target info to the robot's info
    targetRotation = SwerveSubsystem.getInstance().getPose().getRotation();
    targetPos = SwerveSubsystem.getInstance().getPose().getTranslation();
    //initialize vars
    numSamples = 0;
  }

  public void resetTagAtBootup() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // if (alliance.isEmpty() || alliance.get() == Alliance.Blue) {
    //   
    // }
    reset();
  }

  /**
   * Command for removing photon data and changing the target id to look for.
   * @param tagid
   * the id that the subsystem will now look for
   * @return
   * the command
   */
  public Command getResetCommand(){
    return runOnce(() -> reset());
  }
  
  //publishes yaw
  public Rotation2d getYaw (){
    return targetRotation;
  }

  /**
   * Command that does nothing until photonSubsystem has enough data to track a target
   * @param tagid
   * resets the subsystem and its id
   * @return
   * the command
   */
  public Command getWaitForDataCommand(){
    return new FunctionalCommand(() -> reset(), ()->{}, (interrupted) ->{}, ()->hasData(), this);
  }

  public Translation2d getTargetPos(){
    return targetPos;
  }

  public Rotation2d getTargetRotation(){
    return targetRotation;
  }

  public Rotation2d getTargetRobotHeading(){
    return targetRobotHeading;
  }

  public boolean hasData() {
    //if data is the max that the filters hold
    return(numSamples >= PhotonConfig.maxNumSamples);
  }

  private double range(double y) {
    y = Math.toRadians(y);
    y += PhotonConfig.CAMERA_PITCH.getRadians();

    int id_array = id - 3;

    if (id_array < 0) {
      return 0;
    }else if(id_array>= Config.PhotonConfig.APRIL_HEIGHTS.length){
      return 0;
    }

    return (Config.PhotonConfig.APRIL_HEIGHTS[id_array]-PhotonConfig.CAMERA_HEIGHT)/Math.tan(y);
  }

  private PhotonTrackedTarget biggestTarget(List<PhotonTrackedTarget> targets) {
    PhotonTrackedTarget biggestTarget=null;

    double tallest = 0;
    for (PhotonTrackedTarget t:targets) {
      List<TargetCorner> corners = t.getDetectedCorners();
      double sizeY = corners.get(1).y-corners.get(3).y;
      if (sizeY > tallest) {
        tallest = sizeY;
        biggestTarget = t;
      }
    }
    return (biggestTarget);
  }

  private PhotonTrackedTarget getSpeakerTarget(List<PhotonTrackedTarget> targets) {
    PhotonTrackedTarget target=null;

    for (PhotonTrackedTarget t:targets) {
       int targetId = t.getFiducialId();
            if (targetId == 7 )//|| targetId == 4)
            {
              //Do something with this target.. Get the Pose2d, etc.
              //publish the yaw to the network table
              pubSpeakerYaw.accept(t.getYaw());

              target = t;
              break;
            }
      
    }
    return (target);
  }

  private Pose2d convertToField(double range, Rotation2d yaw, Pose2d odometryPose) {
    Rotation2d fieldOrientedTarget = yaw.rotateBy(odometryPose.getRotation());
    Translation2d visionXY = new Translation2d(range, yaw);
    Translation2d robotRotated = visionXY.rotateBy(PhotonConfig.cameraOffset.getRotation());
    Translation2d robotToTargetRELATIVE = robotRotated.plus(PhotonConfig.cameraOffset.getTranslation());
    Translation2d robotToTarget = robotToTargetRELATIVE.rotateBy(odometryPose.getRotation());
    return new Pose2d(robotToTarget.plus(odometryPose.getTranslation()), fieldOrientedTarget);
  }

  @Override
  public void periodic() {

    Pose2d fieldToTarget = null;
    
    // Must be set by 2D or 3D mode;
    //@todo: test 2D and 3D mode both
    PhotonPipelineResult result = camera1.getLatestResult();
    //to test...
    //PhotonPipelineResult result = camera1.getAllUnreadResults().get(0);
    if (result.getTimestampSeconds() == recentTimeStamp){
      //--System.out.println("time stamp is stale");
      return;
    }
    recentTimeStamp = result.getTimestampSeconds();

    if(result.hasTargets()==false)
    { 
     // System.out.println("result does not have any target");
      return;
    }

    //if (PhotonConfig.USE_3D_TAGS) 
    {     
      //Get the best target
      PhotonTrackedTarget target = result.getBestTarget();

      if (target == null)
      {
        System.out.println("Best target is null");
        return;
      }

      bestTagId = target.getFiducialId();
      
      //@todo: validate bestTagId based on red or blue. May not need to check red or blue
      //if not valid tagId, return
      Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(bestTagId);
      Pose3d tagIdPose;
      if (tagPose.isEmpty())
      {
        return;
      }
      else
      {
        tagIdPose = tagPose.get();
        bestTagHeading = tagIdPose.getRotation().toRotation2d();
        targetRobotHeading = bestTagHeading.plus(Rotation2d.fromDegrees(180));

        pubBestTagId.accept(bestTagId);
        pubBestTagHeading.accept(bestTagHeading.getDegrees());
        pubTargetRobotHeading.accept(targetRobotHeading.getDegrees());
      }

      //get the swerve pose at the time that the result was gotten
      Optional<Pose2d> optPose= SwerveSubsystem.getInstance().getPoseAtTimestamp(result.getTimestampSeconds());
      //for security reasons
      if (optPose.isEmpty()){
        //System.out.println("the odometryPose is empty");
        return;
      }
      else
      {
        Pose2d odometryPose = optPose.get();

        Transform3d robotToTarget3d = PhotonConfig.cameraTransform.plus(target.getBestCameraToTarget());
        Transform2d robotToTarget = new Transform2d(robotToTarget3d.getTranslation().toTranslation2d(), robotToTarget3d.getRotation().toRotation2d());

        // Map the position of the tag relative to the current odometry pose with latency compensation
        fieldToTarget = odometryPose.plus(robotToTarget);

        pub3DTagsDebugMsg.accept("Transform2d from robot to tag: " + robotToTarget.toString());
      } 
   
    } 
    
    if (fieldToTarget != null) {
      //update rolling averages
      targetPos = new Translation2d(
          filterX.calculate(fieldToTarget.getX()),
          filterY.calculate(fieldToTarget.getY()));
      targetRotation = Rotation2d.fromDegrees(filteryaw.calculate(fieldToTarget.getRotation().getDegrees()));
      numSamples++;

    //   //publish to networktables
    //   pubSetPoint.accept(new double[]{targetPos.getX(), targetPos.getY(), targetRotation.getRadians()});
    // }
  }
 
  public Command saveImagesIntakeCameraCommand() {
    Timer timer = new Timer();
    timer.start();
    return Commands.run(() -> {
      if (timer.hasElapsed(0.4)) {
        timer.restart();
        intakeCameraInputSaveImgEntry.set(intakeCameraInputSaveImgEntry.get() + 1);
      }
    });
  }
}
