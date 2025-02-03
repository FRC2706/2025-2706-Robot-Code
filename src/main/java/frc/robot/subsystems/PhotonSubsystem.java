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
  private DoublePublisher pubRange, pubYaw;
  public DoublePublisher pubSpeakerRange, pubSpeakerYaw;
  private IntegerPublisher pubSetTagId;
  private StringPublisher pub3DTagsDebugMsg;
  private PhotonCamera camera1;
  private Translation2d targetPos;
  private Rotation2d targetRotation;
  private LinearFilter filteryaw = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private LinearFilter filterX = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private LinearFilter filterY = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private int numSamples;
  private double recentTimeStamp = 0;

  private IntegerEntry intakeCameraInputSaveImgEntry;

  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator photonPoseEstimator;

  public static PhotonSubsystem getInstance(){
    if (instance == null){
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
    pubSetPoint = photonTable.getDoubleArrayTopic("PhotonAprilPoint").publish(PubSubOption.periodic(0.02));
    pubRange = photonTable.getDoubleTopic("Range").publish(PubSubOption.periodic(0.02));
    pubYaw = photonTable.getDoubleTopic("Yaw").publish(PubSubOption.periodic(0.02));
    pubSpeakerRange = photonTable.getDoubleTopic("SpeakerRange").publish(PubSubOption.periodic(0.02));
    pubSpeakerYaw = photonTable.getDoubleTopic("SpeakerYaw").publish(PubSubOption.periodic(0.02));
    pub3DTagsDebugMsg = photonTable.getStringTopic("3DTagsDebugMsg").publish(PubSubOption.periodic(0.02));
    SmartDashboard.putData("command reset id",Commands.runOnce(()->reset()));

    reset();

    try {
      aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
      photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhotonConfig.cameraTransform);
    } catch (Exception e) {
      aprilTagFieldLayout = null;
      PhotonConfig.USE_3D_TAGS = false;
      DriverStation.reportError("Merge's PhotonSubsystem failed to create the apriltag layout. Using 2D apriltags instead of 3D.", false);
    }

    // Intake camera snapping photons
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
    id = desiredId;
  }

    // if (alliance.isEmpty() || alliance.get() == Alliance.Blue) {
    //   reset(8);
    // } else {
    //   reset(4);
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

    //todo: 
  //multiple tags. For now, only tag 8

/**
 * this is the command for going to a specific position based off of an apriltag during teleoperated,------------------------       
 * 
 * during Auto, please use the positions with PhotonMoveToTarget
 * @param spacePositions
 * a PhotonPositions object from the config file
 * @return
 * the command to run
 */

 
 //why this command is in the subsystem?
  public Command getAprilTagCommand(PhotonPositions spacePositions, CommandXboxController driverStick, boolean neverEnd){
    Command moveToTargetCommands;
    if (spacePositions.hasWaypoint) {
      moveToTargetCommands = Commands.sequence(
        new PhotonMoveToTarget(spacePositions.waypoint, spacePositions.direction,true, false),
        new PhotonMoveToTarget(spacePositions.destination, spacePositions.direction, false, neverEnd)
      );
    } else {
      moveToTargetCommands = new PhotonMoveToTarget(spacePositions.destination, spacePositions.direction, false, neverEnd);
    }

    return Commands.sequence(
      // new ProxyCommand(getWaitForDataCommand(spacePositions.id)), // Proxy this command to prevent PhotonSubsystem requirement conflicting with PhotonMoveToTarget's requirements
      new ScheduleCommand(new RumbleJoystick(driverStick, RumbleType.kBothRumble, 0.7, 1, true)),
      new ProxyCommand(moveToTargetCommands.withName("ProxiedSwerveAprilTagCommand")) // Proxy this command to prevent SwerveSubsystem requirement conflicting with WaitForDataCommand's requirements
    );
  }

  public Translation2d getTargetPos(){
    return targetPos;
  }

  public Rotation2d getTargetRotation(){
    return targetRotation;
  }

  public boolean hasData() {
    //if data is the max that the filters hold
    return(numSamples >= PhotonConfig.maxNumSamples);
  }

  @Override
  public void periodic() {
    
    Pose2d fieldToTarget = null;
    // Must be set by 2D or 3D mode
    PhotonPipelineResult result = camera1.getLatestResult();
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
      // Optional<EstimatedRobotPose> optEstPose = photonPoseEstimator.update();
      // if (optEstPose.isEmpty()) {
      //   pub3DTagsDebugMsg.accept("EmptyEstimatedRobotPose"); 
      //   return;
      // }

      // // Ensure the desired tag is set to 3, 4, 7 or 8
      // if (!PhotonConfig.ALLOWED_TAGS_3D.contains(id)) {
      //   pub3DTagsDebugMsg.accept("Desired tag set to " + id);
      //   return;
      // }

      // // Create a list of tags seen
      // ArrayList<Integer> tagsInFrame = new ArrayList<>();
      // for (PhotonTrackedTarget target : optEstPose.get().targetsUsed) {
      //   tagsInFrame.add(target.getFiducialId()); //************************************** */
      // }
         
      // // Todo: add all the reef tags, human player station tags
      // if (! ( tagsInFrame.contains(7) || tagsInFrame.contains(8))) {
      //   pub3DTagsDebugMsg.accept("expected tags not in frame. Tags in frame: " + tagsInFrame.toString());
      //     return;
      // }

      // Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(id);
      // if (tagPose.isEmpty()) {
      //   pub3DTagsDebugMsg.accept("Cannot get apriltag pose");
      //   return;
      // }

      // // Grab the pose from when the image was taken to compensate for how much the robot has moved since the image was taken
      // Optional<Pose2d> odometryPose = SwerveSubsystem.getInstance().getPoseAtTimestamp(optEstPose.get().timestampSeconds);
      // if (odometryPose.isEmpty()) {
      //   pub3DTagsDebugMsg.accept("No odometry pose at timestamp: " + optEstPose.get().timestampSeconds);
      //   return;
      // }

      // // Create a transform that maps the change in Pose between the robot estimated position and the true tag position
      // Transform2d robotToTarget = new Transform2d(optEstPose.get().estimatedPose.toPose2d(), tagPose.get().toPose2d());

      // // Map the position of the tag relative to the current odometry pose with latency compensation
      // fieldToTarget = odometryPose.get().plus(robotToTarget);

      // pub3DTagsDebugMsg.accept("Got pose. Transform2d from robot to tag: " + robotToTarget.toString());
      // end of the old stuff
      //=================================================================================================
      //new stuff

      //Get the best target
      PhotonTrackedTarget target = result.getBestTarget();

      if (target == null)
      {
        System.out.println("Best target is null");
        return;
      }

      //todo check target's ID here. Should accept all Reef's IDs and Human player station IDs.
      //for now, any detected best tag is fine
      // if(target.getFiducialId() != id)
      // {
      //   System.out.println("Best target ID: " + target.getFiducialId());
      //   return;
      // }

      //todo: use the best tag's ID to get its angle in the field.
        // Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(id);
      // if (tagPose.isEmpty()) {
      //   pub3DTagsDebugMsg.accept("Cannot get apriltag pose");
      //   return;
      // }  

      //get the swerve pose at the time that the result was gotten
      Optional<Pose2d> optPose= SwerveSubsystem.getInstance().getPoseAtTimestamp(result.getTimestampSeconds());
      //for security reasons
      if (optPose.isEmpty()){
        System.out.println("the odometryPose is empty");
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

      //publish to networktables
      pubSetPoint.accept(new double[]{targetPos.getX(), targetPos.getY(), targetRotation.getRadians()});
    }
  }

  public Command saveImagesIntakeCameraCommand() {
    Timer timer = new Timer();
    timer.start();
    return Commands.run(() -> {
      if (timer.hasElapsed(0.4)) {
        timer.restart();
        intakeCameraInputSaveImgEntry.set(intakeCameraInputSaveImgEntry.get() + 1);
      
      }
    }
  }
}

