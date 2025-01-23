// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class ErrorTrackingSubsystem extends SubsystemBase {

    ArrayList<SparkMax> motors = new ArrayList<>();
    ArrayList<StringPublisher> motorPublishers = new ArrayList<>();
    ArrayList<GenericEntry> statusTabEntries = new ArrayList<>();
    ArrayList<GenericEntry> errorsTabEntries = new ArrayList<>();
    int currentMotor = 0;
    NetworkTable errorPublish;
    ShuffleboardTab statusTab;
    ShuffleboardTab errorsTab;

    private static ErrorTrackingSubsystem instance;
    public static ErrorTrackingSubsystem getInstance(){
        if(instance == null){
            instance = new ErrorTrackingSubsystem();
        }
        return instance;
    }
    /** Creates a new ErrorTrackingSubsystem. */
    public ErrorTrackingSubsystem() {
        errorPublish = NetworkTableInstance.getDefault().getTable("SparkMax/Errors"); // Errors will be sent to NetworkTables
        statusTab = Shuffleboard.getTab("SparkMax Status"); // Status will be displayed as a boolean variable (whether the spark max is ok)
        errorsTab = Shuffleboard.getTab("SparkMax Errors");
    }

    public ShuffleboardTab getStatusTab() {
        return statusTab;
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (!motors.isEmpty()) { // Ensure motor list is not empty!
            Warnings warnings = motors.get(currentMotor).getStickyWarnings();
            motors.get(currentMotor).clearFaults(); // we need to make sure to clear the bus, so it doesn't just add up
            StringPublisher publisher = motorPublishers.get(currentMotor);
            String faultWords = faultWordToString(warnings); // the faults are marked as IDs and must be converted to strings
            publisher.set(faultWords); // send to networktables
            GenericEntry statusEntry = statusTabEntries.get(currentMotor); // Gets status entry from shuffleboard
            statusEntry.setBoolean(faultWords.isEmpty());
            GenericEntry errorsEntry = errorsTabEntries.get(currentMotor); // Gets errors tab entry from shuffleboard
            errorsEntry.setString(faultWords);
            if (currentMotor < motors.size() - 1 ) {
                currentMotor++; // Next, we will check the next motor, and we will keep incrementing it to not crowd the bus.
            } else {
                currentMotor = 0;
            }
        }
    }

    /**
     * Function to register a new SparkMax to track errors from.
     * @param motor A SparkMax object (the motor).
     */
    public void register(SparkMax motor) {
        statusTabEntries.add(statusTab
                .add(Integer.toString(motor.getDeviceId()), false)
                .withPosition(( (motors.size() % 9)), motors.size() / 9)
                .withSize(1, 1).getEntry());
        errorsTabEntries.add(errorsTab
                .add(Integer.toString(motor.getDeviceId()), "")
                .withPosition(3 * (motors.size() % 3), motors.size() / 3)
                .withSize(3, 1).getEntry());
        motors.add(motor);
        motorPublishers.add(errorPublish.getStringTopic(Integer.toString(motor.getDeviceId())).publish());
    }

    /**
     * Function to convert a "fault word" to a string.
     * @param warnings A Warnings object containing the warnings.
     * @return A string containing the faults as a string.
     */
    public static String faultWordToString(Warnings warnings) {

        String warningString = "";

        if (warnings.brownout) {
            warningString += "Brownout ";
        }
        if (warnings.overcurrent) {
            warningString += "Overcurrent";
        }
        if (warnings.escEeprom) {
            warningString += "EscEeprom";
        }
        if (warnings.extEeprom) {
            warningString += "ExtEeprom";
        }
        if (warnings.sensor) {
            warningString += "Sensor";
        }
        if (warnings.stall) {
            warningString += "Stall";
        }
        if (warnings.hasReset) {
            warningString += "Hasreset";
        }
        if (warnings.other) {
            warningString += "Other";
        }

        if (warningString.isEmpty()) {
            return "";
        }

        return warningString;
    }

}

// TODO Log using a StringLogEntry
