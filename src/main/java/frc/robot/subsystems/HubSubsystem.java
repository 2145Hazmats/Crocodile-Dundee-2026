// MICHALS MOMENT HAS COME AGAIN!

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubSubsystem extends SubsystemBase {

  RobotContainer m_RobotContainer;

    public enum MatchShift {
        AUTO, TRANSITION, SHIFT1, SHIFT2, SHIFT3, SHIFT4, ENDGAME
    }

    private MatchShift currentShift = MatchShift.AUTO;
    private double matchTime = 160.0;
    private double timeToNextSwitch = 0.0;
    private boolean warning = false;

    // Boolean hub states
    private boolean redHubActive = true;
    private boolean blueHubActive = true;

    public HubSubsystem(RobotContainer robotContainer) {
      m_RobotContainer = robotContainer;
    }

    @Override
    public void periodic() {
        updateMatchTime();
        updateShift();
        updateHubStatus();
        updateDashboard();
        updateRumble();
    }

    private void updateMatchTime() {
      if(DriverStation.isEnabled())
        matchTime -= 0.020;
    }

    private void updateShift() {
      if (matchTime > 140) currentShift = MatchShift.AUTO;
      else if (matchTime > 130) currentShift = MatchShift.TRANSITION;
      else if (matchTime > 105) currentShift = MatchShift.SHIFT1;
      else if (matchTime > 80) currentShift = MatchShift.SHIFT2;
      else if (matchTime > 55) currentShift = MatchShift.SHIFT3;
      else if (matchTime > 30) currentShift = MatchShift.SHIFT4;
      else if (matchTime < 30) currentShift = MatchShift.ENDGAME;
    }

    private void updateHubStatus() {
        switch (currentShift) {
            case AUTO, TRANSITION, ENDGAME -> {
                redHubActive = true;
                blueHubActive = true;
            }
            case SHIFT1, SHIFT3 -> {
                redHubActive = false;
                blueHubActive = true;
            }
            case SHIFT2, SHIFT4 -> {
                redHubActive = true;
                blueHubActive = false;
            }
        }

        double nextBoundary =
                (matchTime > 140) ? 140 :
                (matchTime > 130) ? 130 :
                (matchTime > 105) ? 105 :
                (matchTime > 80) ? 80 :
                (matchTime > 55) ? 55 :
                (matchTime > 30) ? 30 : 0;

        timeToNextSwitch = matchTime - nextBoundary;
        warning = timeToNextSwitch <= 2.0;
    }

    private void updateDashboard() {
        SmartDashboard.putBoolean("Hub/HubActive", isHubActive());
        SmartDashboard.putString("Hub/Shift", currentShift.name());
        SmartDashboard.putNumber("Hub/MatchTime", Timer.getMatchTime());
        SmartDashboard.putNumber("Hub/TimeToNextSwitch", timeToNextSwitch);
        SmartDashboard.putBoolean("Hub/Warning", warning);
        SmartDashboard.putString("game specific message", DriverStation.getGameSpecificMessage());

    }

    private void updateRumble(){
      m_RobotContainer.setControllerRumbles(warning ? 1 : 0);
    }

    public boolean isHubActive() {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      // If we have no alliance, we cannot be enabled, therefore no hub.
      if (alliance.isEmpty()) {
        return false;
      }
      // Hub is always enabled in autonomous.
      if (DriverStation.isAutonomousEnabled()) {
        return true;
      }
      // At this point, if we're not teleop enabled, there is no hub.
      if (!DriverStation.isTeleopEnabled()) {
        return false;
      }

      // We're teleop enabled, compute.
      String gameData = DriverStation.getGameSpecificMessage();

      // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
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
    
      // Shift is active for blue if red won auto, or red if blue won auto.
      boolean shift1Active = switch (alliance.get()) {
        case Red -> !redInactiveFirst;
        case Blue -> redInactiveFirst;
      };

      switch(currentShift){
        case TRANSITION, AUTO, ENDGAME: return true;
        case SHIFT1, SHIFT3: return shift1Active;
        case SHIFT2, SHIFT4: return !shift1Active;
        default: return true;
      }
}

    // --- Public API ---
    public boolean isRedHubActive() { return redHubActive; }
    public boolean isBlueHubActive() { return blueHubActive; }
    public MatchShift getShift() { return currentShift; }
    public boolean isWarning() { return warning; }
    public double getTimeToNextSwitch() { return timeToNextSwitch; }
}