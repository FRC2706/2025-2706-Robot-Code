// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;

public class CoralIntakeSubsystem extends SubsystemBase {

  private static CoralIntakeSubsystem instance = null; // static object that contains all movement controls

  private static final MotorType motorType = MotorType.kBrushless; // defines brushless motortype

  //left motor controller
  private final SparkMax m_leftMotor;  
  //right motor controller
  private final SparkMax m_rightMotor;
  //left motor configuration
  private SparkMaxConfig m_leftMotor_config;
  //right motor configuration
  private SparkMaxConfig m_rightMotor_config;

  public static CoralIntakeSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.CoralIntakeSubsystem);
      instance = new CoralIntakeSubsystem();
    }
    return instance;
  }

  //@todo: the CAN IDs
  int leftMotorCanID = 38;
  int rightMotorCanID = 34;

  /** Creates a new CoralIntakeSubsystem. */
  public CoralIntakeSubsystem() {
    m_leftMotor = new SparkMax(leftMotorCanID, motorType); // creates SparkMax left motor controller
    m_leftMotor_config = new SparkMaxConfig(); //create configuration of the left motor controller

    m_rightMotor = new SparkMax(rightMotorCanID, motorType); // creates SparkMax right motor controller
    m_rightMotor_config = new SparkMaxConfig(); //create configuration of the right motor controller

    //configure the left motor controller  
    //m_leftMotor.setCANTimeout(Config.CANTIMEOUT_MS);
    m_leftMotor_config.inverted(true); //@todo: to adjust
    m_leftMotor_config.idleMode(IdleMode.kBrake);
     
    m_leftMotor_config.smartCurrentLimit(60);
    m_leftMotor_config.voltageCompensation(10);
 
    m_leftMotor.configure(m_leftMotor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    //m_leftMotor.setCANTimeout(0);

    //configure rigth motor controller
    //m_rightMotor.setCANTimeout(Config.CANTIMEOUT_MS);
    m_rightMotor_config.inverted(true);//@todo: to adjust
    m_rightMotor_config.idleMode(IdleMode.kBrake);
     
    m_rightMotor_config.smartCurrentLimit(60);
    m_rightMotor_config.voltageCompensation(10);

    m_rightMotor.configure(m_leftMotor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    //m_rightMotor.setCANTimeout(0);
    
    //@todo: to add encoder and PIDs

    ErrorTrackingSubsystem.getInstance().register(m_leftMotor);
    ErrorTrackingSubsystem.getInstance().register(m_rightMotor);

  }

  public void startIntakePercent(double leftPercentOutput, double rightPercentOutput){
    m_leftMotor.set(leftPercentOutput);
    m_rightMotor.set(rightPercentOutput);
  }

  public void startIntakeRPM(double leftRPM, double rightRPM)
  {
    //@todo: to add
  }

  public void stop() 
  {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}