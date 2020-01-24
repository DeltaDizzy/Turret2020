/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.lib.util.Rotation2d;

/**
 * A container class to specify the shooter angle. It contains the desired
 * range, the turret angle, and the computer vision's track's ID.
 */
public class ShooterAimingParameters {
    double range;
    Rotation2d turret_angle;
    int track_id;

    public ShooterAimingParameters(double range, Rotation2d turret_angle) {
        this.range = range;
        this.turret_angle = turret_angle;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getTurretAngle() {
        return turret_angle;
    }
}
