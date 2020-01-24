package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * A basic framework for the control board Like the drive code, one instance of
 * the ControlBoard object is created upon startup, then other methods request
 * the singleton ControlBoard instance.
 * 
 * @see The The ControlBoard is a programmatic representation of the various buttons and axes used to control the robot. 
 * @see The The Superstructure queries if specific inputs are active and controls subsystems based on this.
 */
public class ControlBoard {
    private static ControlBoard mInstance = new ControlBoard();

    public static ControlBoard getInstance() {
        return mInstance;
    }

  //  private final Joystick mThrottleStick;
  //  private final Joystick mTurnStick;
    private final XboxController mDriver;
    private final XboxController mTurretControl;
    private boolean turretManual;

    private ControlBoard() {
      //  mThrottleStick = new Joystick(0);
      //  mTurnStick = new Joystick(1);
       mDriver = new XboxController(0);
       mTurretControl = new XboxController(1);
       turretManual = false;

    }

    // DRIVER CONTROLS
    public double getThrottle() {
        double throttle=0;
        
        if(mDriver.getTriggerAxis(Hand.kRight) > 0.05) {
            throttle=mDriver.getTriggerAxis(Hand.kRight);			
        }else if(mDriver.getTriggerAxis(Hand.kLeft) > 0.05) {
            throttle=-mDriver.getTriggerAxis(Hand.kLeft);
           
        }else {
            throttle=0;
        }
       // if(getDriveInverted()&&throttle!=0){
            //turn=turn;
       //     throttle=-throttle;
       // }
       return throttle;
    }

    public double getTurn() {
        return mDriver.getX(Hand.kLeft);
    }

    public boolean getQuickTurn() {
        return mDriver.getAButton();
    }

    public boolean getTractionControl() {
        return mDriver.getBButton();
    }


    // TURRET CONTROLS

    /** 
     * Switches the turret between Manual and Automatic Control.
     * @return True if the turret swivel is controlled manually, False if it is controlled by the Limelight.
    */
    public Boolean getTurretToggle()
    {
        if (mTurretControl.getXButton()) {
            if (!turretManual) 
            {
                turretManual = true;  
            }
            else if (turretManual) {
                turretManual = false;
            }
        }
        return turretManual;
    }
    
    /**
     * Commands the turret to turn if it is in Manual mode.
     * @see ControlBoard.getTurretToggle()
     * @return The control demand to send to the turret swivel controller.
     */
    public double getTurretTurn()
    {
        return mTurretControl.getX(Hand.kLeft);
    }
}
