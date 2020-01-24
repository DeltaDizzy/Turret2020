/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

/**
 * Add your docs here.
 */
import frc.robot.subsystems.ShooterAimingParameters;
import frc.robot.subsystems.Superstructure;

/**
 * Action for aiming the turret at a specified target
 */
public class PointTurretAction implements Action {
    private ShooterAimingParameters mHint;
    private boolean mIsDone;
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    public PointTurretAction(ShooterAimingParameters hint) {
        mHint = hint;
        mIsDone = false;
    }

    @Override
    public boolean isFinished() {
        return (mIsDone && mSuperstructure.HasTarget());
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        mSuperstructure.clearTurretManualPositionSetpoint();
    }

    @Override
    public void start() {
        mSuperstructure.setTurretManualPositionSetpoint(mHint);
        mIsDone = true;
    }
}