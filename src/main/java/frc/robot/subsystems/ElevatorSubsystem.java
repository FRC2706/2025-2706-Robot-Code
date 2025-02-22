// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import frc.robot.Config.ElevatorSetPoints;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem instance = null; // static object that contains all movement controls

  private static final MotorType motorType = MotorType.kBrushless; // defines brushless motortype

  //elevator motor controller
  private final SparkMax m_elevator;  
  //elevator motor configuration
  private SparkMaxConfig m_elevator_config;
  //PID close loop controller
  private SparkClosedLoopController m_pidController;
  //relative encoder
  private RelativeEncoder m_encoder;

  //@todo: add the switch...

  //@todo: elevator CAN ID
  int elevatorCanID = 15;

  // network table entry
  private final String m_tuningTable = "Elevator/ElevatorTuning";
  private final String m_dataTable = "Elevator/ElevatorData";
   // network table entries
  private DoubleEntry m_ElevatorPSubs;
  private DoubleEntry m_ElevatorISubs;
  private DoubleEntry m_ElevatorDSubs;
  private DoubleEntry m_ElevatorIzSubs;
  private DoubleEntry m_ElevatorFFSubs;
  
  private DoublePublisher m_targetPosPub;
  private DoublePublisher m_currentPosPub;

  private double elevatorCurrentTarget = ElevatorSetPoints.IDLE.position;

  public static ElevatorSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.ElevatorSubsystem);
      instance = new ElevatorSubsystem();
    }
    return instance;
  }

  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevator = new SparkMax(elevatorCanID, motorType); // creates SparkMax motor controller
    m_elevator_config = new SparkMaxConfig(); //create configuration of the motor controller

    m_pidController = m_elevator.getClosedLoopController();
    m_encoder = m_elevator.getEncoder();

    //configure the elevator motor controller   
    m_elevator_config.inverted(false)
                     .idleMode(IdleMode.kBrake)
                     .smartCurrentLimit(50)
                     .voltageCompensation(12);
    
    //set up the network entry
    NetworkTable ElevatorTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTable);
    m_ElevatorPSubs = ElevatorTuningTable.getDoubleTopic("P").getEntry(0.1 );
    m_ElevatorISubs = ElevatorTuningTable.getDoubleTopic("I").getEntry(0.0);
    m_ElevatorDSubs = ElevatorTuningTable.getDoubleTopic("D").getEntry(0.0);
    m_ElevatorIzSubs = ElevatorTuningTable.getDoubleTopic("IZone").getEntry(0.0);
    m_ElevatorFFSubs = ElevatorTuningTable.getDoubleTopic("FF").getEntry(0.0);

    m_ElevatorPSubs.setDefault(0.5);
    m_ElevatorISubs.setDefault(0.0);
    m_ElevatorDSubs.setDefault(0.0);
    m_ElevatorIzSubs.setDefault(0.0);
    m_ElevatorFFSubs.setDefault(0.0);

    m_elevator_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(m_ElevatorPSubs.get(), m_ElevatorISubs.get(), m_ElevatorDSubs.get())
                                //.pid(0.1, 0.0, 0.0)
                                //.iZone(0.02)
                                .velocityFF(0.003)
                                .outputRange(-1,1)
                                .maxMotion.maxVelocity(1000)
                                          .maxAcceleration(1000)
                                          .allowedClosedLoopError(0.25);


    //encoder configuration
    // double r = 0.02; //unit meter
    // double gear_ratio = 1.0;
    // double positionConvFactor = 1.0;//2*Math.PI*r*gear_ratio;
    // m_elevator_config.encoder.positionConversionFactor(positionConvFactor) //in meters
    //                          .velocityConversionFactor(positionConvFactor/60.0); // in m/s

    //m_elevator_config.signals.primaryEncoderPositionPeriodMs(20);

    // m_elevator_config.encoder.inverted(false)
    //                         .positionConversionFactor(2*Math.PI*r*gear_ratio);
                        
    
    //normal config
    //m_elevator.configure(m_elevator_config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    //persist memory
    m_elevator.configure(m_elevator_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    ErrorTrackingSubsystem.getInstance().register(m_elevator);

    //reset encoder position
    m_encoder.setPosition(0);

    

    NetworkTable ElevatorDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);
    m_currentPosPub = ElevatorDataTable.getDoubleTopic("CurrentPostition").publish(PubSubOption.periodic(0.02));
    m_targetPosPub = ElevatorDataTable.getDoubleTopic("TargetPosition").publish(PubSubOption.periodic(0.02));
    
  }

  public void startElevatorPercent(double percentOutput){
    m_elevator.set(percentOutput);
    
  }

  public void setElevatorRPM(double rpm)
  {
    m_pidController.setReference(rpm, ControlType.kVelocity);
    
  }

  public void setElevatorPosition(double position)
  {
    m_pidController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    //m_pidController.setReference(position, ControlType.kMAXMOtionPositionControl, ClosedLoopSlot.kSlot0);
   
  }

  public void resetEncoderPosition()
  {
    m_encoder.setPosition(0.0);
  }

  public void updateEncoderPosition(double newPosition)
  {
    m_encoder.setPosition(newPosition);
  }

  public double getCurrentPosition()
  {
    return m_encoder.getPosition();
  }
  public void stop() 
  {
    m_elevator.stopMotor();
  }

  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setElevatorSetpointCommand(ElevatorSetPoints setpoint) {
    return this.runOnce(
        () -> {
          elevatorCurrentTarget = setpoint.position;
          }
        );
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //******************************************* */
    setElevatorPosition(elevatorCurrentTarget);

    //@todo: when the switch is detected, reset the position
    //m_encoder.setPosition(0.0);

    //update the network table
    m_targetPosPub.accept(elevatorCurrentTarget);
    m_currentPosPub.accept(getCurrentPosition());
  }
}
