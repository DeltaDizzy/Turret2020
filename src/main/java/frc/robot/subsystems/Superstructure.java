/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import frc.robot.Constants;
import frc.robot.ControlBoard;
import frc.robot.GoalTracker;
import frc.robot.Limelight;
import frc.robot.RobotState;
import frc.robot.loops.Loop;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.Rotation2d;

import frc.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The overarching superclass containing all components of the superstructure:
 * the computer vision, flywheel, hood, hood roller, and intake. This
 * coordinates the entire firing sequence, from when the ball is intaked to when
 * it is fired.
 */
public class Superstructure extends Subsystem {

    static Superstructure mInstance = new Superstructure();

    public static Superstructure getInstance() {
        return mInstance;
    }

    /**
     * Drives actual state, all outputs should be dictated by this state
     */
    private enum SystemState {
        REENABLED, // The shooter has just been enabled (and may have previously
                   // been disabled from any state)
        DEPLOYED_AND_HOMING_HOOD, // The shooter is deployed and the hood is
                                  // homing
        STOWED_OR_STOWING, // The shooter is stowed (or stowing)
        UNSTOWING, // The shooter is in the process of unstowing
        LOADING, // The shooter is bringing in the ball
        SPINNING_AIM, // The shooter is auto aiming
        FIRING_AIM, // The shooter is firing in auto aim mode
        DEPLOYED, // The shooter is deployed but idle
        KEEP_SPINNING, // The shooter is deployed and running but not aiming
        DEPLOYED_RETURNING_TO_SAFE // The shooter is preparing to stow
    }

    /**
     * Drives state changes. Outputs should not be decided based on this enum.
     */
    public enum WantedState {
        WANT_TO_DEPLOY, // The user is okay with leaving the hood up
        WANT_TO_STOW, // The user wants to stow the shooter
        WANT_TO_AIM, // The user wants to auto aim
        WANT_TO_BATTER, // The user wants to use batter mode
        WANT_TO_KEEP_SPINNING // The user wants to keep the wheel spinning
    }

    /**
     * Orthogonal to the shooter state is the state of the firing mechanism.
     * Outputs should not be decided based on this enum.
     */
    public enum WantedFiringState {
        WANT_TO_HOLD_FIRE, // The user does not wish to fire
        WANT_TO_FIRE_NOW, // The user wants to fire now, regardless of readiness
        WANT_TO_FIRE_WHEN_READY // The user wants to fire as soon as we achieve readiness
    }

    private WantedState mWantedState = WantedState.WANT_TO_DEPLOY;
    private WantedFiringState mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;

    private double mCurrentRangeForLogging;
    private double mCurrentAngleForLogging;
    private SystemState mSystemStateForLogging;
    private boolean mTuningMode = false;

    private double mTurretManualScanOutput = 0;
    private ShooterAimingParameters mTurretManualSetpoint = null;
    int mConsecutiveCyclesOnTarget = 0;
    int mNumShotsFired = 0;

    private List<ShooterAimingParameters> mCachedAimingParams = new ArrayList<>();

    Turret mTurret = new Turret();
    RobotState mRobotState = RobotState.getInstance();
    Drive mDrive = Drive.getInstance();
    private final TimeDelayedBoolean mHasBallDelayedBoolean = new TimeDelayedBoolean();

    NetworkTable mShooterTable = NetworkTableInstance.getDefault().getTable("shooter");

    Loop mLoop = new Loop() {

        private SystemState mSystemState = SystemState.REENABLED;

        // Every time we transition states, we update the current state start
        // time and the state changed boolean (for one cycle)
        private double mCurrentStateStartTime;
        private boolean mStateChanged;

        @Override
        public void onStart() {
            synchronized (Superstructure.this) {
                mWantedState = WantedState.WANT_TO_DEPLOY;
                mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;
                mCurrentStateStartTime = Timer.getFPGATimestamp();
                mSystemState = SystemState.REENABLED;
                mStateChanged = true;
                mTurretManualSetpoint = null;
            }
        }

        @Override
        public void onLoop() {
            synchronized (Superstructure.this) {
                double now = Timer.getFPGATimestamp();
                SystemState newState;
                switch (mSystemState) {
                case REENABLED: // robot just reenabled
                    newState = handleReenabled();
                    break;
                case DEPLOYED_AND_HOMING_HOOD:
                    newState = handleDeployedAndHomingHood();
                    break;
                case STOWED_OR_STOWING: // stowing hood/intake or just stowed
                    newState = handleStowedOrStowing();
                    break;
                case UNSTOWING: // unstowing hood
                    newState = handleUnstowing(now, mCurrentStateStartTime);
                    break;
                case LOADING: // loading balls
                    newState = handleLoading(now, mCurrentStateStartTime);
                    break;
                case SPINNING_AIM: // spinning flywheel but not firing
                    newState = handleSpinningAim(now, mStateChanged);
                    break;
                case FIRING_AIM: // spinning flywheel and firing
                    newState = handleShooting(mSystemState, now, mCurrentStateStartTime);
                case DEPLOYED_RETURNING_TO_SAFE:
                    newState = handleReturningToSafe(now, mCurrentStateStartTime);
                    break;
                default:
                    System.out.println("Unexpected shooter state: " + mSystemState);
                    newState = SystemState.DEPLOYED_RETURNING_TO_SAFE;
                }

                if (newState != mSystemState) {
                    System.out.println("Shooter state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                    mCurrentStateStartTime = now;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
                mSystemStateForLogging = mSystemState;
            }
        }

        @Override
        public void onStop() {
            synchronized (Superstructure.this) {
                mTurret.stop();

            }
        }
    };
//TODO: if SPINNING_AIM, move turret, 
    private Superstructure() {
    }

    public Loop getLoop() {
        return mLoop;
    }

    public synchronized void setWantedState(WantedState newState) {
        mWantedState = newState;
    }

    @Override
    public void outputToSmartDashboard() {
        mTurret.outputToSmartDashboard();
        SmartDashboard.putNumber("current_range", mCurrentRangeForLogging);
        SmartDashboard.putNumber("current_angle", mCurrentAngleForLogging);
        SmartDashboard.putString("shooter_state", "" + mSystemStateForLogging);
    }

    @Override
    public synchronized void stop() {

        mTurret.stop();
    }

    @Override
    public synchronized void zeroSensors() {
        mTurret.zeroSensors();
        mCachedAimingParams.clear();
    }

    public synchronized void resetTurretAtMax() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardMaxTurretAngle));
    }

    public synchronized void resetTurretAtMin() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardMinTurretAngle));
    }

    public synchronized void zeroTurret() {
        mTurret.reset(new Rotation2d());
    }

    /**
     * Sets the manual turret output ONLY when the turret is auto-aiming without
     * a visible target
     */
    public synchronized void setTurretManualScanOutput(double output) {
        mTurretManualScanOutput = output;
    }

    public synchronized void setTurretManualPositionSetpoint(ShooterAimingParameters parameters) {
        mTurretManualSetpoint = parameters;
    }

    public synchronized void clearTurretManualPositionSetpoint() {
        mTurretManualSetpoint = null;
    }

    public synchronized void setWantsToFireNow() {
        mWantedFiringState = WantedFiringState.WANT_TO_FIRE_NOW;
    }

    public synchronized void setWantsToFireWhenReady() {
        mWantedFiringState = WantedFiringState.WANT_TO_FIRE_WHEN_READY;
    }

    public synchronized void setWantsToHoldFire() {
        mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;
    }

    public synchronized int getNumShotsFired() {
        return mNumShotsFired;
    }

    public Turret getTurret() {
        return mTurret;
    }

    public ShooterAimingParameters getAimFromControls()
    {
        ShooterAimingParameters param = new ShooterAimingParameters(0.0, new Rotation2d(mTurret.txEntry.getDouble(0.0), mTurret.tyEntry.getDouble(0.0), false));
        return param;
    }
    public synchronized void setTuningMode(boolean tuning_on) {
        mTuningMode = tuning_on;
    }

    private synchronized SystemState handleReenabled() {
        mTurret.setDesiredAngle(new Rotation2d());
        double now = Timer.getFPGATimestamp();
        return handleReturningToSafe(now, now);
    }

    private synchronized SystemState handleDeployedAndHomingHood() {
        mTurret.setDesiredAngle(new Rotation2d());

        return true ? SystemState.DEPLOYED : SystemState.DEPLOYED_AND_HOMING_HOOD;
    }

    private synchronized SystemState handleStowedOrStowing() {
        mTurret.setDesiredAngle(new Rotation2d());

        switch (mWantedState) {
        case WANT_TO_AIM: // fallthrough
        case WANT_TO_DEPLOY:
        case WANT_TO_KEEP_SPINNING:
            return SystemState.UNSTOWING;
        case WANT_TO_STOW: // fallthrough
        default:
            // Nothing to do, these outputs are redundant
            return SystemState.STOWED_OR_STOWING;
        }
    }

    private synchronized SystemState handleUnstowing(double now, double stateStartTime) {
        mTurret.setDesiredAngle(new Rotation2d());

        // State transitions
        boolean isDoneUnstowing = now - stateStartTime > Constants.kHoodUnstowToFlywheelSpinTime;
        switch (mWantedState) {
        case WANT_TO_DEPLOY:
            if (!isDoneUnstowing) {
                return SystemState.UNSTOWING;
            } else {
                return SystemState.DEPLOYED;
            }
        case WANT_TO_KEEP_SPINNING:
            if (!isDoneUnstowing) {
                return SystemState.UNSTOWING;
            } else {
                return SystemState.KEEP_SPINNING;
            }
        case WANT_TO_AIM:
        case WANT_TO_BATTER: // fallthrough
            if (!isDoneUnstowing) {
                return SystemState.UNSTOWING;
            } else {
                return SystemState.LOADING;
            }
        case WANT_TO_STOW: // fallthrough
        default:
            return SystemState.STOWED_OR_STOWING;
        }
    }

    private synchronized SystemState handleLoading(double now, double stateStartTime) {
        mTurret.setDesiredAngle(new Rotation2d());

        switch (mWantedState) {
        case WANT_TO_AIM: // fallthrough
        case WANT_TO_DEPLOY:
        case WANT_TO_KEEP_SPINNING:
            boolean isDoneLoading = now - stateStartTime > Constants.kLoadingTime;
            if (isDoneLoading) {
                if (mWantedState == WantedState.WANT_TO_AIM) {
                    return SystemState.SPINNING_AIM;
                } else if (mWantedState == WantedState.WANT_TO_KEEP_SPINNING) {
                    return SystemState.KEEP_SPINNING;
                } else {
                    return SystemState.DEPLOYED;
                }
            } else {
                return SystemState.LOADING;
            }
        case WANT_TO_STOW:
        default:
            return SystemState.DEPLOYED_RETURNING_TO_SAFE;
        }
    }

    private synchronized SystemState handleSpinningAim(double now, boolean isFirstCycle) {
        // State transition
        switch (mWantedState) {
        case WANT_TO_AIM:
            if (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_NOW
                    || (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_WHEN_READY)) {
                System.out.println("Turret Angle (desired): " + mTurret.getSetpoint() + ", (actual): " + mTurret.getAngle());
                return SystemState.FIRING_AIM;
            } else {
                mTurret.setDesiredAngle(getAimFromControls().turret_angle);
                return SystemState.SPINNING_AIM;
            }
        case WANT_TO_DEPLOY:
            return SystemState.DEPLOYED;
        case WANT_TO_KEEP_SPINNING:
            return SystemState.KEEP_SPINNING;
        case WANT_TO_STOW: // fallthrough
        default:
            return SystemState.DEPLOYED_RETURNING_TO_SAFE;
        }
    }

    private synchronized SystemState handleSpinningBatter(double now) {
        mTurret.setDesiredAngle(new Rotation2d());

        // state transitions
        switch (mWantedState) {
        case WANT_TO_AIM:
            return SystemState.SPINNING_AIM;
        case WANT_TO_DEPLOY:
            return SystemState.DEPLOYED;
        case WANT_TO_KEEP_SPINNING:
            return SystemState.KEEP_SPINNING;
        case WANT_TO_STOW: // fallthrough
        default:
            return SystemState.DEPLOYED_RETURNING_TO_SAFE;
        }
    }


    private synchronized SystemState handleReturningToSafe(double now, double start_time) {
        mTurret.setDesiredAngle(new Rotation2d());

        switch (mWantedState) {
        case WANT_TO_AIM:
            return SystemState.SPINNING_AIM;
        case WANT_TO_DEPLOY:
            return SystemState.DEPLOYED;
        case WANT_TO_KEEP_SPINNING:
            return SystemState.KEEP_SPINNING;
        default:
            if ((mTurret.isSafe()) || (now - start_time > Constants.kStowingOverrideTime)) {
                return SystemState.STOWED_OR_STOWING;
            } else {
                return SystemState.DEPLOYED_RETURNING_TO_SAFE;
            }
        }
    }

    private synchronized SystemState handleShooting(SystemState state, double now, double stateStartTime) {
        if (state == SystemState.FIRING_AIM) {
            autoAim(now, true);
        }

        if (now - stateStartTime < Constants.kShootActuationTime) {
            return state;
        } else {
            mNumShotsFired++;
            switch (mWantedState) {
            case WANT_TO_AIM:
                return SystemState.SPINNING_AIM;
            case WANT_TO_DEPLOY:
                return SystemState.DEPLOYED;
            case WANT_TO_KEEP_SPINNING:
                return SystemState.KEEP_SPINNING;
            case WANT_TO_STOW: // fallthrough
            default:
                return SystemState.DEPLOYED_RETURNING_TO_SAFE;
            }
        }
    }
    private List<ShooterAimingParameters> getCurrentAimingParameters(double now) {
        List<ShooterAimingParameters> params = mRobotState.getAimingParameters(now,
                new GoalTracker.TrackReportComparator(Constants.kTrackReportComparatorStablityWeight,
                        Constants.kTrackReportComparatorAgeWeight, Constants.kTrackReportComparatorSwitchingWeight, now));
        mCachedAimingParams = params;
        return params;

    }

    public List<ShooterAimingParameters> getCachedAimingParams() 
    {
        return mCachedAimingParams;
    }

    /**
     * 
     * @param now - Current timestamp
     * @param allow_changing_tracks - true
     */
    private void autoAim(double now, boolean allow_changing_tracks) {
        List<ShooterAimingParameters> aimingParameters = getCurrentAimingParameters(now);
        if (aimingParameters.isEmpty() && (allow_changing_tracks || mTurretManualSetpoint != null)) {
            // Manual search
            if (mTurretManualSetpoint != null) {
                mTurret.setDesiredAngle(mTurretManualSetpoint.getTurretAngle());
            } else {
                mTurret.setOpenLoop(mTurretManualScanOutput);
            }
        } else {
            // Pick the target to aim at
            boolean has_target = false;
            for (ShooterAimingParameters param : aimingParameters) {
                double turret_angle_degrees = param.getTurretAngle().getDegrees(); // current angle
                if (turret_angle_degrees >= Constants.kSoftMinTurretAngle
                        && turret_angle_degrees <= Constants.kSoftMaxTurretAngle
                        && param.getRange() >= Constants.kAutoAimMinRange
                        && param.getRange() <= Constants.kAutoAimMaxRange
                        && (allow_changing_tracks)) {
                    // This target works
                    mTurret.setDesiredAngle(param.getTurretAngle());
                    mCurrentAngleForLogging = param.getTurretAngle().getDegrees();
                    mCurrentRangeForLogging = param.getRange();
                    has_target = true;
                    break;
                }
            }
        }
    }

    public synchronized boolean HasTarget() {
        if (Limelight.getTargetValid() == 1) 
        {
            return true;
        }
        else
        {
            return false;
        }    
    }
}
