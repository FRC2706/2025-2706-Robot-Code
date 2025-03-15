// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.SubsystemChecker;
import frc.lib.lib2706.SubsystemChecker.SubsystemType;
import frc.robot.Config;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase{
    private SparkMax m_intake;
    private SparkMaxConfig m_intake_config;

    private DigitalInput frontSensor;//  -> 0 
    private DigitalInput centerSensor;// -> 1
    private DigitalInput backSensor;//   -> 2

    private Debouncer frontSensorDebouncer;
    private Debouncer centerSensorDebouncer;
    private Debouncer backSensorDebouncer;
    private Debouncer backSensorLongDebouncer;

    private BooleanPublisher frontSensorPub;
    private BooleanPublisher centerSensorPub;
    private BooleanPublisher backSensorPub;
    private BooleanPublisher backSensorLongPub;
    private StringPublisher statesPub;

    private boolean frontSensorResult;
    private boolean centerSensorResult;
    private boolean backSensorResult;
    private boolean backSensorLongResult;

    private static IntakeSubsystem instance;
    public static IntakeSubsystem getInstance() {
        if (instance == null)
        {
            SubsystemChecker.subsystemConstructed(SubsystemType.IntakeSubsystem);
            instance = new IntakeSubsystem();
        }
            
        return instance;
    }

    private IntakeSubsystem() {
        System.out.println("[Init]Creating Intake");
        m_intake = new SparkMax(Config.Intake.INTAKE, MotorType.kBrushless);
        m_intake_config = (SparkMaxConfig) new SparkMaxConfig()
                .inverted(true)
                .smartCurrentLimit(70)
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(10);

        m_intake.configure(m_intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        frontSensor = new DigitalInput(Config.Intake.frontSensor);
        centerSensor = new DigitalInput(Config.Intake.centerSensor);
        backSensor = new DigitalInput(Config.Intake.backSensor);

        frontSensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        centerSensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        backSensorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        backSensorLongDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kBoth);

        NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
        statesPub = intakeTable.getStringTopic("Intake's Current State").publish(PubSubOption.periodic(0.02));
        frontSensorPub = intakeTable.getBooleanTopic("front sensor result").publish(PubSubOption.periodic(0.02));
        centerSensorPub = intakeTable.getBooleanTopic("center sensor result").publish(PubSubOption.periodic(0.02));
        backSensorPub = intakeTable.getBooleanTopic("back sensor result").publish(PubSubOption.periodic(0.02));
        backSensorLongPub = intakeTable.getBooleanTopic("back sensor result").publish(PubSubOption.periodic(0.02));

        ErrorTrackingSubsystem.getInstance().register(m_intake);



        // Must be the last thing in the constructor
        //burnFlash(); // Broken in 2025
    }

    /**
     * Save the configurations from flash to EEPROM.
     */
    /*private void burnFlash() {
        try {
        Thread.sleep(200);
        } 
        catch (Exception e) {}

        m_intake.burnFlash();
    }*/ // Broken in 2025

    public boolean isFrontSensorActive(){
        return frontSensorResult;
    }

    public boolean isCenterSensorActive(){
        return centerSensorResult;
    }
    
    public boolean isBackSensorActive(){
        return backSensorResult;
    }

    public boolean isBackSensorLongActive(){
        return backSensorLongResult;
    }

    public void setVoltage(double voltage){
        m_intake.setVoltage(voltage);
    }

    public void stop() 
    {
      m_intake.stopMotor();
    }

  

    
    /*---------------------------Commands---------------------------*/

    @Override
    public void periodic() {
        frontSensorResult = frontSensorDebouncer.calculate(!frontSensor.get());
        centerSensorResult = centerSensorDebouncer.calculate(!centerSensor.get());
        backSensorResult = backSensorDebouncer.calculate(!backSensor.get());
        backSensorLongResult = backSensorLongDebouncer.calculate(!backSensor.get());

        frontSensorPub.accept(frontSensorResult);
        centerSensorPub.accept(centerSensorResult);
        backSensorPub.accept(backSensorResult);
        backSensorLongPub.accept(backSensorLongResult);

    }
}
