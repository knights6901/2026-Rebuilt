package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum GameState {
    INACTIVE,
    ACTIVE,
    TRANSITION;

    /** Returns the current status of the alliance hub based on the FMS timing. */
    public static GameState get() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty())
            return INACTIVE;

        if (DriverStation.isAutonomousEnabled())
            return ACTIVE;
        if (!DriverStation.isTeleopEnabled())
            return INACTIVE;

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty())
            return INACTIVE;

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return ACTIVE;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        GameState active = shift1Active ? ACTIVE : INACTIVE;

        // Transition shift, hub is active.
        if (matchTime > 135)
            return TRANSITION;
        else if (matchTime > 130)
            return active;

        // shift 1
        else if (matchTime > 110)
            return active;
        else if (matchTime > 105)
            return TRANSITION;

        // shift 2
        else if (matchTime > 85)
            return active;
        else if (matchTime > 80)
            return TRANSITION;

        // shift 3
        else if (matchTime > 60)
            return active;
        else if (matchTime > 55)
            return TRANSITION;

        // shift 4
        else if (matchTime > 35)
            return active;
        else if (matchTime > 30)
            return TRANSITION;

        // endgame
        else if (matchTime > 5)
            return ACTIVE;

        return TRANSITION;
    }
}
