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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;

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
  int elevatorCanID = 22;

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

    //configure the left motor controller
    m_elevator.setCANTimeout(Config.CANTIMEOUT_MS);
    
    m_elevator_config.inverted(false)
                     .idleMode(IdleMode.kBrake)
                     .smartCurrentLimit(20)
                     .voltageCompensation(6);
    
    //closed loop configuration
    m_elevator_config.closedLoop.minOutput(-1.0);
    m_elevator_config.closedLoop.maxOutput(1.0);
    m_elevator_config.closedLoop.p(0.0002, ClosedLoopSlot.kSlot0);
    m_elevator_config.closedLoop.i(0.0, ClosedLoopSlot.kSlot0);
    m_elevator_config.closedLoop.d(0.0, ClosedLoopSlot.kSlot0);
    m_elevator_config.closedLoop.velocityFF(0.003, ClosedLoopSlot.kSlot0);

    //encoder configuration
    double r = 0.02; //unit meter
    double gear_ratio = 1.0;
    m_elevator_config.encoder.inverted(false)
                            .positionConversionFactor(1.0*2*Math.PI*r*gear_ratio);
                        
    
    //normal config
    //m_elevator.configure(m_elevator_config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    //persist memory
    m_elevator.configure(m_elevator_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    m_elevator.setCANTimeout(0);

    ErrorTrackingSubsystem.getInstance().register(m_elevator);

    //reset encoder position
    m_encoder.setPosition(0);
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
    m_pidController.setReference(position, ControlType.kPosition);
   
  }

  public void resetEncoderPosition()
  {
    m_encoder.setPosition(0.0);
    //m_pidController.setReference(0, ControlType.kPosition);
  }

  public void stop() 
  {
    m_elevator.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //@todo: when the switch is detected, reset the position
    //m_encoder.setPosition(0.0);
  }
}
