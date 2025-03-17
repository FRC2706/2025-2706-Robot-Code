// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;

public class AlgaeSubsystem extends SubsystemBase {

    private static AlgaeSubsystem instance = null; // static object that contains all movement controls

    private static final MotorType motorType = MotorType.kBrushless; // defines brushless motortype

    //algae motor controller
    private final SparkMax m_algaeMotor;
    //algae motor configuration
    private SparkMaxConfig m_algaeMotor_config;
    private RelativeEncoder m_algae_encoder;

    private final String m_dataTable = "Algae";
     private DoublePublisher m_currentPositionPub;

    public static AlgaeSubsystem getInstance() {
        if (instance == null) {
            SubsystemChecker.subsystemConstructed(SubsystemType.AlgaeSubsystem);
            instance = new AlgaeSubsystem();
        }
        return instance;
    }

    /** Creates a new AlgaeSubsystem. */
    public AlgaeSubsystem() {
        m_algaeMotor = new SparkMax(Config.CANID.ALGAE_REMOVER, motorType); // creates SparkMax algae motor controller
        m_algaeMotor_config = new SparkMaxConfig(); //create configuration of the algae motor controller

        m_algae_encoder = m_algaeMotor.getEncoder();

        //configure the algae motor controller  
        //m_algaeMotor.setCANTimeout(Config.CANTIMEOUT_MS);
        m_algaeMotor_config.inverted(true); //@todo: to adjust
        m_algaeMotor_config.idleMode(IdleMode.kBrake);

        m_algaeMotor_config.smartCurrentLimit(60);
        m_algaeMotor_config.voltageCompensation(10);

        //@todo: tune the limits
        m_algaeMotor_config.softLimit.reverseSoftLimit(-160)
                                   .reverseSoftLimitEnabled(true)
                                   .forwardSoftLimit(0) 
                                   .forwardSoftLimitEnabled(true);


        m_algaeMotor.configure(m_algaeMotor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        //reset encoder
        m_algae_encoder.setPosition(0);
        //@todo: to add encoder and PIDs

        ErrorTrackingSubsystem.getInstance().register(m_algaeMotor);

        NetworkTable ElevatorDataTable = NetworkTableInstance.getDefault().getTable(m_dataTable);
        m_currentPositionPub = ElevatorDataTable.getDoubleTopic("CurrentPosition").publish(PubSubOption.periodic(0.02));
    }

    public void setPercent(double percentOutput){
        m_algaeMotor.set(percentOutput);
    }


    public void stop()
    {
        m_algaeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_currentPositionPub.accept(m_algae_encoder.getPosition());
    }
}