/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

 public class Limelight{
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry tx = table.getEntry("tx");
    static NetworkTableEntry ty = table.getEntry("ty");
    static NetworkTableEntry ta = table.getEntry("ta");
    static NetworkTableEntry tv = table.getEntry("tv");
    //read values periodically

    public static double getHAngle()
    {
        double horizontalAngle = tx.getDouble(0.0);
        return horizontalAngle;
    }

    public static double getVAngle()
    {
        double verticalAngle = ty.getDouble(0.0);
        return verticalAngle;
    }
    
    public static double getArea()
    {
        double targetArea = ta.getDouble(0.0);
        return targetArea;
    }

    public static double getTargetValid()
    {
        double targetValid = tv.getDouble(0.0);
        return targetValid;
    }


}