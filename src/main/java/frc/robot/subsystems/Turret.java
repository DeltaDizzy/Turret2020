/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.lib.util.Rotation2d;
import frc.lib.util.drivers.Talon.CANTalonFactory;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight;

/**
 * The Turret subsystem controls the direction the ball is fired. On the Turret
 * assembly is the Hood and Flywheel. The Turret can only rotate within 240
 * degrees (+/- 120), and mechanical bumper switches indicate when the
 * mechanical limits are reached. This is part of the Superstructure superclass.
 * 
 * The ball is first picked up with the Intake then is fed to the Flywheel with
 * the HoodRoller. The Turret controls the direction that the ball is fired at.
 * Finally, the Hood controls the output angle and trajectory of the shot.
 * 
 * @see Flywheel
 * @see Hood
 * @see HoodRoller
 * @see Intake
 * @see Superstructure
 */
public class Turret extends Subsystem {
    private TalonSRX talon;
    /**
     * Current angle targeted by the turret, specified in Degrees
     */
    private double setpoint;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry txEntry;
    NetworkTableEntry tyEntry;
    NetworkTableEntry tvEntry;
    NetworkTableEntry taEntry;
    NetworkTableEntry rangeEntry;
    boolean target;

    Turret() {
        // The turret has one Talon.
        talon = CANTalonFactory.createTalon(7, false, NeutralMode.Brake, FeedbackDevice.QuadEncoder, 0, false);
        talon = CANTalonFactory.tuneLoops(talon, 7, Constants.kTurretKp, Constants.kTurretKi, Constants.kTurretKd, Constants.kTurretKf);
        talon = CANTalonFactory.setupSoftLimits(talon, true, 1826, true, 5);
        talon = CANTalonFactory.setupHardLimits(talon, LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, false, 
            LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, true);

        txEntry = table.getEntry("tx");
        tyEntry = table.getEntry("ty");
        tvEntry = table.getEntry("tv");
        taEntry = table.getEntry("ta");


    }

    /**
     * Polls the Limelight to see if it has identified a target
     */
    synchronized Boolean GetTarget()
    {
        Boolean t = false;
        if (Limelight.getTargetValid() == 1) 
        {
            t = true;
        }
        if (Limelight.getTargetValid() == 0)
        {
            t = false;
        }
        return t;
    }

    // Set the desired angle of the turret (and put it into position control
    // mode if it isn't already).
    synchronized void setDesiredAngle(Rotation2d angle) {
        setpoint = angle.getDegrees();
        talon.set(ControlMode.Position, angle.getRadians() / (2 * Math.PI * Constants.kTurretRotationsPerTick));
    }

    // Manually move the turret (and put it into vbus mode if it isn't already).
    synchronized void setOpenLoop(double speed) {
        talon.set(ControlMode.PercentOutput, speed);
    }

    // Tell the Talon it is at a given position.
    synchronized void reset(Rotation2d actual_rotation) {
        talon.setSelectedSensorPosition(Integer.parseInt(String.valueOf(actual_rotation.getRadians() / (2 * Math.PI * Constants.kTurretRotationsPerTick))));
    }
    

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians(Constants.kTurretRotationsPerTick * talon.getSelectedSensorPosition() * 2 * Math.PI);
    }

    public synchronized double getSetpoint() {
        return setpoint * Constants.kTurretRotationsPerTick * 360.0;
    }

    private synchronized double getError() {
        return getAngle().getDegrees() - getSetpoint();
    }

    // We are "OnTarget" if we are in position mode and close to the setpoint.
    public synchronized boolean isOnTarget() {
        return (talon.getControlMode() == ControlMode.Position
                && Math.abs(getError()) < Constants.kTurretOnTargetTolerance);
    }

    /**
     * @return If the turret is within its mechanical limits and in the right
     *         state.
     */
    public synchronized boolean isSafe() {
        return (talon.getControlMode() == ControlMode.Position && talon.getClosedLoopError() == 0 && Math.abs(
                getAngle().getDegrees() * Constants.kTurretRotationsPerTick * 360.0) < Constants.kTurretSafeTolerance);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("turret_error", getError());
        SmartDashboard.putNumber("turret_angle", getAngle().getDegrees());
        SmartDashboard.putNumber("turret_setpoint", getSetpoint());
        SmartDashboard.putBoolean("turret_on_target", isOnTarget());
    }

    @Override
    public void zeroSensors() {
        reset(new Rotation2d());
    }
}