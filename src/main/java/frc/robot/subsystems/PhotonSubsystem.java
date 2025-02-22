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
  private DoublePublisher pubRange, pubYaw;
  public DoublePublisher pubSpeakerRange, pubSpeakerYaw;
  private IntegerPublisher pubSetTagId;
  private StringPublisher pub3DTagsDebugMsg;
  //private IntegerArrayPublisher pub3DTagIdsArray;
  private StringPublisher pub3DTargetTags;
  private IntegerEntry subOverrideTagID;
  private PhotonCamera camera1;
  private Translation2d targetPos;
  private Rotation2d targetRotation;
  private LinearFilter filteryaw = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private LinearFilter filterX = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private LinearFilter filterY = LinearFilter.movingAverage(PhotonConfig.maxNumSamples);
  private int numSamples;
  private int id;
  private double recentTimeStamp = 0;
  private ArrayList<Boolean> aprilTagsId = new ArrayList<>();

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
    pubSetPoint = photonTable.getDoubleArrayTopic("PhotonAprilPoint").publish(PubSubOption.periodic(0.02));
    pubRange = photonTable.getDoubleTopic("Range").publish(PubSubOption.periodic(0.02));
    pubYaw = photonTable.getDoubleTopic("Yaw").publish(PubSubOption.periodic(0.02));
    pubSpeakerRange = photonTable.getDoubleTopic("SpeakerRange").publish(PubSubOption.periodic(0.02));
    pubSpeakerYaw = photonTable.getDoubleTopic("SpeakerYaw").publish(PubSubOption.periodic(0.02));
    pubSetTagId = photonTable.getIntegerTopic("SetTagId").publish();
    pub3DTagsDebugMsg = photonTable.getStringTopic("3DTagsDebugMsg").publish(PubSubOption.periodic(0.02));
    pub3DTargetTags = photonTable.getStringTopic("3DTagsTargetID's").publish(PubSubOption.periodic(0.02));
    //pub3DTagIdsArray = photonTable.getIntegerArrayTopic("3DTagsTargetID's").publish(PubSubOption.periodic(0.02));
    subOverrideTagID = photonTable.getIntegerTopic("OVERIDEID").getEntry(-1);
    subOverrideTagID.setDefault(-1);
    SmartDashboard.putData("command reset id",Commands.runOnce(()->reset((int)subOverrideTagID.get())));

    reset(-1);

    aprilTagsId.add(0, false);
    aprilTagsId.add(1, false);
    aprilTagsId.add(2, false);
    aprilTagsId.add(3, false);

    try {
      aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();
      //photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, PhotonConfig.cameraTransform);
    } catch (Exception e) {
      aprilTagFieldLayout = null;
      PhotonConfig.USE_3D_TAGS = false;
      DriverStation.reportError("Merge's PhotonSubsystem failed to create the apriltag layout. Using 2D apriltags instead of 3D.", false);
    }

    // Intake camera snapping photons
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable(PhotonConfig.frontCameraName);
    intakeCameraInputSaveImgEntry = intakeTable.getIntegerTopic("inputSaveImgCmd").getEntry(0);
  }

  public void reset(int desiredId) {
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

  public void resetTagAtBootup() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == Alliance.Blue) {
      reset(7);
    } else {
      reset(4);
    }
  }

  /**
   * Command for removing photon data and changing the target id to look for.
   * @param tagid
   * the id that the subsystem will now look for
   * @return
   * the command
   */
  public Command getResetCommand(int tagid){
    return runOnce(() -> reset(tagid));
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
  public Command getWaitForDataCommand(int tagid){
    return new FunctionalCommand(() -> reset(tagid), ()->{}, (interrupted) ->{}, ()->hasData(), this);
  }

/**
 * this is the command for going to a specific position based off of an apriltag during teleoperated,------------------------       
 * 
 * during Auto, please use the positions with PhotonMoveToTarget
 * @param spacePositions
 * a PhotonPositions object from the config file
 * @return
 * the command to run
 */
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
    // List<PhotonTrackedTarget> targets = camera1.getLatestResult().targets;
    // targets.get(0).getYaw();
    // targets.get(0).getFiducialId();

    
  

    // if (fieldToTarget != null) {
    //   //update rolling averages
    //   targetPos = new Translation2d(
    //       filterX.calculate(fieldToTarget.getX()),
    //       filterY.calculate(fieldToTarget.getY()));
    //   targetRotation = Rotation2d.fromDegrees(filteryaw.calculate(fieldToTarget.getRotation().getDegrees()));
    //   numSamples++;

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
