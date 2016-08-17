package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.connectivity.fri.ClientCommandMode;
import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRIJointOverlay;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseExecutionService;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Creates a FRI Session.
 */
public class _ROSFRIServer extends RoboticsAPIApplication
{
    private enum ControlMode {
    	POSITION,
    	JOINT_IMPEDANCE,
    	GRAVITY_COMP  // Not intended for sending commands through FRI, just reading them
    }

    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private int _controlFrequency;
    private IMotionControlMode _iiwaControlMode;
    private PositionHold _overlayMotion;
    private IMotionContainer _motionContainer;
    private FRISession _friSession;
    private SunriseExecutionService _sunriseService;
    private ControlMode _externalControlMode;
    private FRIJointOverlay _jointOverlay;

    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        _sunriseService = (SunriseExecutionService)_lbrController.getExecutionService();

        // **********************************************************************
        // *** CUSTOMIZE PARAMETERS                                           ***
        // **********************************************************************
        _clientName = "192.170.10.1"; // change to FRIClient's IP address
        _externalControlMode = ControlMode.GRAVITY_COMP;
        _controlFrequency = 500; // 200
        // Set the uniform joint stiffness from the teach pendant under process data
    }

    public void switchMode()
    {
    	switch(_externalControlMode) {
    		case POSITION:
            	getLogger().info("Switching to POSITION mode");
            	_jointOverlay = new FRIJointOverlay(_friSession, ClientCommandMode.POSITION);
            	_iiwaControlMode = new PositionControlMode();
            	_overlayMotion = new PositionHold(_iiwaControlMode, -1, TimeUnit.SECONDS);
            	//_shortPositionHold = new PositionHold(_iiwaControlMode, 5, TimeUnit.MILLISECONDS);
    			break;
    		case JOINT_IMPEDANCE:
    			getLogger().info("Switching to JOINT_IMPEDANCE mode");
    			_jointOverlay = new FRIJointOverlay(_friSession, ClientCommandMode.TORQUE);
    			double stiffness = getApplicationData().getProcessData("jointImpedance").getValue();
    			_iiwaControlMode = new JointImpedanceControlMode(stiffness, stiffness, stiffness, stiffness, stiffness, stiffness, stiffness);
            	_overlayMotion = new PositionHold(_iiwaControlMode, -1, TimeUnit.SECONDS);
            	//_shortPositionHold = new PositionHold(_iiwaControlMode, 5, TimeUnit.MILLISECONDS);
    			break;
    		case GRAVITY_COMP:
    			getLogger().info("Switching to GRAVITY_COMP mode");
    			_jointOverlay = new FRIJointOverlay(_friSession, ClientCommandMode.NO_COMMAND_MODE); // TODO this last part is ok?
    			_iiwaControlMode = new JointImpedanceControlMode(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            	_overlayMotion = new PositionHold(_iiwaControlMode, -1, TimeUnit.SECONDS);
            	//_shortPositionHold = new PositionHold(_iiwaControlMode, 5, TimeUnit.MILLISECONDS);
    			break;
    		default:
    			getLogger().error("Unknown mode requested");
    	}
    }
    @Override
    public void run()
    {
    	//int selectedMode = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Enter control mode", "Position", "Gravity Comp");
    	int selectedMode = 0;
		switch (selectedMode)
		{
			case 0: // Position control
		        _externalControlMode = ControlMode.POSITION;
		        break;
			case 1: // Gravity comp
		        _externalControlMode = ControlMode.GRAVITY_COMP;
		        break;
		    default:
				getLogger().info("Unknown mode");
				return;
		}


        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);
        double SECOND_TO_MILLISECOND = 1000.0;
        //friConfiguration.setSendPeriodMilliSec(5);
        friConfiguration.setSendPeriodMilliSec((int) ((1.0 / _controlFrequency) * SECOND_TO_MILLISECOND));

        getLogger().info("Creating FRI connection to ROS node at " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        _friSession = new FRISession(friConfiguration);

        // Load desired mode
        switchMode();

        // wait until FRI session is ready to switch to command mode
        try
        {
        	int wait_time = 60;
            getLogger().info("Waiting for FRI connection up to " + wait_time + " s");
            _friSession.await(wait_time, TimeUnit.SECONDS);
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            _friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");

        // Determine if we are sending commands
        if (_externalControlMode != ControlMode.GRAVITY_COMP)
        {
        	// async move with overlay
        	getLogger().info("Starting async move with overlay");
        	_motionContainer = _lbr.moveAsync(_overlayMotion.addMotionOverlay(_jointOverlay));
        } else {
        	getLogger().info("Not creating overlay (just sending position hold for gravity comp)");
    		_lbr.getFlange().move(positionHold(_iiwaControlMode, -1, TimeUnit.SECONDS));
        }

        // Run until application is stopped (paused)
        while(!_sunriseService.isPaused())
        {
	        try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				getLogger().error("While loop: " + e.toString() );
				//e.printStackTrace();
				break;
			}
	    }

		// Stop any running commands
    	getLogger().info("Stopping running commands");
	    if ((_motionContainer != null) && !_motionContainer.isFinished()) {
	    	getLogger().info("Canceling motion container");
	    	_motionContainer.cancel();
	    }

		getLogger().info("run() has completed");
    }

    @Override
	public void dispose() {
		getLogger().info("dispose()");

	    try {
			// Stop any running commands
	    	getLogger().info("Stopping running commands");
		    if ((_motionContainer != null) && !_motionContainer.isFinished()) {
		    	getLogger().info("Canceling");
		    	_motionContainer.cancel();
		    	//_lbr.move(_shortPositionHold); // Issue a very brief position hold with no overlay
		    }

		    // End FRI
	    	getLogger().info("Closing FRI Session");
	    	_friSession.close();
		}
	    finally {
	    	// It is imperative that you call super.dispose() when you override this method
	    	getLogger().info("Super disposing");
	        super.dispose();
	    }
	}



    /**
     * main.
     *
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final _ROSFRIServer app = new _ROSFRIServer();
        app.runApplication();
    }

}
