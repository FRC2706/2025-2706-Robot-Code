// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  //Instance Variables
  private SparkMax m_climber;
  private SparkMaxConfig m_climber_config;
  private RelativeEncoder m_encoder;
  //private double targetRPM = CLIMBER_RPM.getValue();
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public double currentPosition = 0;
  public boolean m_bGoodSensors = false;
  private static final ClimberSubsystem INSTANCE_CLIMBER = new ClimberSubsystem();

  /** Creates a new ClimberSubSystem. */
  private ClimberSubsystem() {
    
    if (Config.Climber_CANID.CLIMBER != -1) {
      initializeSubsystem();
    }
    else
    {
      m_climber = null;
    }

    SubsystemChecker.subsystemConstructed(SubsystemType.ClimberSubsystem);

  }

  private void initializeSubsystem() 
  {
    m_climber = new SparkMax(Config.Climber_CANID.CLIMBER, MotorType.kBrushless);
    m_climber_config = new SparkMaxConfig();

    if ( m_climber != null )
    {      
      m_bGoodSensors = true;

      m_climber_config.inverted(false);

      //Set maximum current
      m_climber_config.smartCurrentLimit(40);

      m_climber.configure(m_climber_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      
      ErrorTrackingSubsystem.getInstance().register(m_climber);

    }
  }


    public boolean isAvailable() 
    {
        return m_climber != null;
    }

     /*
     * Returns the singleton instance for the Climber Subsystem
     */
    public static ClimberSubsystem getInstance() {
      if ( INSTANCE_CLIMBER.isAvailable() == true)
        return INSTANCE_CLIMBER;
      else
        return null;
    } 

    //Run the climber

    public void StartClimberRPM(double percentOutput){
        m_climber.set(percentOutput);
    }

    public void stop() 
    {
      m_climber.stopMotor();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

