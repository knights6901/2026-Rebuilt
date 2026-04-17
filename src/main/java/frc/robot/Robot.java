// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private boolean activeStatus = false;
    private boolean currentlyLogging = false;

    private final StringPublisher phaseNamePublisher = NetworkTableInstance.getDefault()
            .getTable("Match Time")
            .getStringTopic("Phase Name")
            .publish();
    private final DoublePublisher phaseTimePublisher = NetworkTableInstance.getDefault()
            .getTable("Match Time")
            .getDoubleTopic("Phase Time")
            .publish();
    private final BooleanPublisher activeShiftPublisher = NetworkTableInstance.getDefault()
            .getTable("QoL")
            .getBooleanTopic("Active")
            .publish();

    // private boolean currentlyLogging = false;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        // bring back after comp
        // DataLogManager.start();
    }

    @Override
    public void robotPeriodic() {
        if (DriverStation.isFMSAttached() && !currentlyLogging) {
            DataLogManager.start();
            currentlyLogging = true;
        } else if (!DriverStation.isFMSAttached() && currentlyLogging) {
            DataLogManager.stop();
            currentlyLogging = false;
        }
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        activeShiftPublisher.set(activeStatus);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        activeStatus = true;

        phaseNamePublisher.set("Auton");
    }

    @Override
    public void autonomousPeriodic() {
        phaseTimePublisher.set(DriverStation.getMatchTime());
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        activeStatus = true;

        phaseNamePublisher.set("Teleop");
    }

    @Override
    public void teleopPeriodic() {
        double matchTime = DriverStation.getMatchTime();
        String phaseName = "Unknown";
        double phaseTime = 0;

        if (DriverStation.isFMSAttached()) {
            String autonWinner = DriverStation.getGameSpecificMessage();
            String alliance = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? "R" : "B";

            boolean validData = autonWinner.length() > 0 && (alliance.equals("R") || alliance.equals("B"));

            if (matchTime > 130) {
                phaseTime = matchTime - 130;
                phaseName = "Transition";
            } else if (matchTime > 105) {
                phaseTime = matchTime - 105;
                phaseName = validData ? (autonWinner.equals(alliance) ? "Inactive" : "Active") : "Error LOL";
                activeStatus = validData ? (autonWinner.equals(alliance) ? false : true) : false;
            } else if (matchTime > 80) {
                phaseTime = matchTime - 80;
                phaseName = validData ? (autonWinner.equals(alliance) ? "Active" : "Inactive") : "Error LOL";
                activeStatus = validData ? (autonWinner.equals(alliance) ? true : false) : false;
            } else if (matchTime > 55) {
                phaseTime = matchTime - 55;
                phaseName = validData ? (autonWinner.equals(alliance) ? "Inactive" : "Active") : "Error LOL";
                activeStatus = validData ? (autonWinner.equals(alliance) ? false : true) : false;
            } else if (matchTime > 30) {
                phaseTime = matchTime - 30;
                phaseName = validData ? (autonWinner.equals(alliance) ? "Active" : "Inactive") : "Error LOL";
                activeStatus = validData ? (autonWinner.equals(alliance) ? true : false) : false;
            } else {
                phaseTime = matchTime;
                phaseName = "!!!!ENDGAME!!!!";
                activeStatus = true;
            }
        } else {
            phaseName = "Testing";
            phaseTime = matchTime;
            activeStatus = true;
        }

        phaseNamePublisher.set(phaseName);
        phaseTimePublisher.set(phaseTime);
    }

    public static boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
