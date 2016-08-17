package application;


import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadInterruptedException;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fri.ClientCommandMode;
import com.kuka.connectivity.fri.FRIConfiguration;
import com.kuka.connectivity.fri.FRIJointOverlay;
import com.kuka.connectivity.fri.FRISession;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISunriseRequestService;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSR;
import com.kuka.roboticsAPI.controllerModel.sunrise.api.SSRFactory;
import com.kuka.roboticsAPI.controllerModel.sunrise.connectionLib.Message;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.IMotionOverlay;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class FRIServer extends RoboticsAPIApplication {
	// Fields for the robot and the controller objects
	private Controller kukaController;
	private LBR lbr;
	
	// Fields for the asynchronous channel communication
	//private String hostName = "172.31.1.147";
	private int portNumber = 49001;
	private ServerSocket serverSocket;
	private Socket asyncSocket;
	private PrintWriter asyncOut;
	private BufferedReader asyncIn;
	
	// Fields for the FRI communication 
	private String friHost = "312.213.321.123";
	private int friPort = 49999;
	private int sendPeriod_ms = 7000;

	// Debug constants
	final int DEBUG_STARTUP 	  = 0x01;
	final int DEBUG_SHUTDOWN 	  = 0x02;
	final int DEBUG_CLEANUP 	  = 0x04;
	final int DEBUG_ASYNCMESSAGES = 0x08;
	final int DEBUG_COMMANDS      = 0x10;
	final int DEBUG_MOTION        = 0x20;
	private int debugLevel;
	
	// SSR constants
    //private final static int REFERENCE_JOINTTORQUESENSOR = 2;
    private final static int SSR_SUCCESS = 1;

	private boolean shutdownServer = false;
	
	private boolean initSuccess = false;
	
    private FRISession friSession;
    private IMotionOverlay motionOverlay;
    private PositionHold overlayMotion;
    private PositionHold shortPositionHold;
    private IMotionControlMode controlMode;
    private IMotionControlMode positionControlMode;
    private IMotionContainer mc;

	private String doStartSession( String friAddress, int friPort, int sendPeriod_ms) {
        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(lbr, friAddress);
        friConfiguration.setPortOnRemote(friPort);
        friConfiguration.setSendPeriodMilliSec(sendPeriod_ms);

        if ((debugLevel & DEBUG_COMMANDS) != 0) {
            getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
            getLogger().info("Creating FRI connection to " + friConfiguration.getPortOnRemote());
            getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                    + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());
        }

        try {
            friSession = new FRISession(friConfiguration);
        } catch (Exception e) {
        	getLogger().error("Failed to create FRI session: " + e.toString());
        	return "SESSION:FAILED";
        }
        return "SESSION:STARTED";
	}
	
	private String doStopSession() {
        // Shut down the FRI session
        if ((debugLevel & DEBUG_COMMANDS) != 0) {
            getLogger().info("doStopSession: Shutting down FRI session");
        }
        if ((mc != null) && !mc.isFinished()) {
        	return "SESSION:MOTIONINPROGRESS";
        }
        try {
    		friSession.close();
    		friSession = null;
        } catch (Exception e) {
        	getLogger().error("doStopSession: Exception closing friSession: " + e);
        	return "SESSION:ERRORONCLOSE";
        }
        if ((debugLevel & DEBUG_COMMANDS) != 0) {
        	getLogger().info("doStopSession: FRI session closed");	
        }
        
        if (friSession != null) {
        	getLogger().info("doStopSession: friSession closed, but not null; setting to null");
        	friSession = null;
        }
        return "SESSION:CLOSED";
	}

	private String doSessionCommand(String[] parsedCommand) {
		String response = "SESSION:UNEXPECTEDCOMMAND";
		if (parsedCommand[1].equals("TICK")) {
			sendPeriod_ms = Integer.parseInt(parsedCommand[2]);
			response = "SESSION:TICK:SET";
			}
		if (parsedCommand[1].equals("HOST")) {
			friHost = parsedCommand[2];
			response = "SESSION:HOST:SET";
			}
		if (parsedCommand[1].equals("PORT")) {
			friPort = Integer.parseInt(parsedCommand[2]);
			response = "SESSION:PORT:SET";
			}
		if (parsedCommand[1].equals("START")) {
			response = doStartSession(friHost, friPort, sendPeriod_ms);
		}
		if (parsedCommand[1].equals("STOP")) {
			response = doStopSession();
		}
		return response;
	}
	
	private String createOverlay(String commandMode) {
		motionOverlay = null;  // Set to null to test if an unexpected overlay type was received
		
        // Creates the specified overlay for use in the main loop
        if ((debugLevel & DEBUG_COMMANDS) != 0) {
        	getLogger().info("Creating overlay");
        }
        
        if (commandMode.equals("POSITION")) {
        	motionOverlay = new FRIJointOverlay(friSession, ClientCommandMode.POSITION);
            if ((debugLevel & DEBUG_COMMANDS) != 0) {
            	getLogger().info("Joint position overlay created");
            }
            
        }
        if (commandMode.equals("WRENCH")) {
        	motionOverlay = new FRIJointOverlay(friSession, ClientCommandMode.WRENCH);
            if ((debugLevel & DEBUG_COMMANDS) != 0) {
            	getLogger().info("Joint wrench overlay created");	
            }
            
        }
        if (commandMode.equals("TORQUE")) {
        	motionOverlay = new FRIJointOverlay(friSession, ClientCommandMode.TORQUE);
            if ((debugLevel & DEBUG_COMMANDS) != 0) {
            	getLogger().info("Joint torque overlay created");
            }
            
        }
        if (motionOverlay == null) {
        	return "COMMANDMODE:UNEXPECTED";
        }
        
        return "COMMANDMODE:"+commandMode;
	}

	private String createOverlayMotion(String controlType) {
		overlayMotion = null;  // Set to null to test if unexpected overlay type was received
		
        // Creates the specified overlay for use in the main loop
        if ((debugLevel & DEBUG_COMMANDS) != 0) {
        	getLogger().info("Creating overlay motion");	
        }
        
        if (controlType.equals("POSITION")) {
        	controlMode = new PositionControlMode();
        	overlayMotion = new PositionHold(controlMode, -1, TimeUnit.SECONDS);
        	shortPositionHold = new PositionHold(controlMode, 5, TimeUnit.MILLISECONDS);
            if ((debugLevel & DEBUG_COMMANDS) != 0) {
            	getLogger().info("Position control mode created");
            }
            
        }
        if (controlType.equals("CART_IMP")) {
        	controlMode = new CartesianImpedanceControlMode();
        	overlayMotion = new PositionHold(controlMode, -1, TimeUnit.SECONDS);
        	shortPositionHold = new PositionHold(controlMode, 5, TimeUnit.MILLISECONDS);
            if ((debugLevel & DEBUG_COMMANDS) != 0) {
            	getLogger().info("Cartesian impedance control mode created");
            }
            
        }
        if (controlType.equals("JOINT_IMP")) {
        	double stiffness = getApplicationData().getProcessData("jointImpedance").getValue();
        	controlMode = new JointImpedanceControlMode(stiffness, stiffness, stiffness, stiffness, stiffness, stiffness, stiffness);
        	overlayMotion = new PositionHold(controlMode, -1, TimeUnit.SECONDS);
        	shortPositionHold = new PositionHold(controlMode, 5, TimeUnit.MILLISECONDS);
            if ((debugLevel & DEBUG_COMMANDS) != 0) {
            	getLogger().info("Joint impedance control mode created");
            }
            
        }
        if (overlayMotion == null) {
        	return "CONTROLMODE:UNEXPECTED";
        }
        
        return "CONTROLMODE:"+controlType;
	}
	
	private String doMotionCommand(String motionCommand) {
		if (motionCommand.equals("START")) {
			if ((mc == null) || mc.isFinished()) {
				// Make sure FRI is in MONITORING_READY before starting the motion
				// Give it 10 ticks
		        try {
		            friSession.await(10*sendPeriod_ms, TimeUnit.MILLISECONDS);
		        }
		        catch (final TimeoutException e) {
		            getLogger().error("processMessages: await timed out: " + e.getLocalizedMessage());
		            return "MOTION:FRISTATE";
	            }
		        
		        // await finished without a timeout exception; issue the motion
				mc = lbr.moveAsync(overlayMotion.addMotionOverlay(motionOverlay));
	    	    if ((debugLevel & DEBUG_MOTION) != 0) {
	    	    	getLogger().info("moveAsync returned");
	    	    }
	    	    if ((mc != null) && !mc.isFinished()) {
		    	    if ((debugLevel & DEBUG_MOTION) != 0) {
	    	    		getLogger().info("mc OK");
	    	    	}
	    	    }
	    	    else {
	    	    	getLogger().error("mc is null or isFinished");
	    	    }
				return "MOTION:STARTED";
			}
			else {
				 return "MOTION:INPROGRESS";
			}
			
		}
		
		if (motionCommand.equals("CANCEL")) {
			if ((debugLevel & DEBUG_COMMANDS) != 0) {
				getLogger().info("About to cancel motion");
			}
			if (mc == null) {
				getLogger().info("mc is null");
				return "MOTION:NOMOTION";
			}
			else {
				if (mc.isFinished()) {
					getLogger().info("motion is finished");
					return "MOTION:MOTIONFINISHED";
				}
				else {
					mc.cancel();

					// Issue a very brief position hold with no overlay
					lbr.move(shortPositionHold);
					
					if (!mc.isFinished()) {
						getLogger().warn("mc canceled, but not isFinished");
					}
					// Wait for the motion to actually complete
					while (!mc.isFinished()) {
						try {
							ThreadUtil.milliSleep(10);
						} catch (ThreadInterruptedException e) {
							// Ignore this exception
							//getLogger().error("Exception on ThreadUtil.milliSleep waiting for motion to terminate:" + e.toString());
						}
					}

					if ((debugLevel & DEBUG_COMMANDS) != 0) {
						getLogger().info("After canceling motion");
					}
					
					return "MOTION:CANCELED";
				}
			}
		}
		
		return "MOTION:UNEXPECTEDCOMMAND";
	}
	
	private String doSSR(String ssrCommand) {
		ISunriseRequestService requestService = (ISunriseRequestService) (kukaController.getRequestService());
        SSR ssr = SSRFactory.createSafetyCommandSSR(Integer.parseInt(ssrCommand));
        Message ssrResponse = requestService.sendSynchronousSSR(ssr);
        int result = ssrResponse.getParamInt(0);
        if (result != SSR_SUCCESS) {
        	if ((debugLevel & DEBUG_COMMANDS) != 0) {
                getLogger().warn("Command did not execute successfully, response = " + result);
        	}
        	return "SSR:FAILED";
        }
        return "SSR:SUCCESS";
	}

	
	public void processMessages() {
		String newCommand = "Empty";
		String response = "Empty";
		String[] parsedCommand;
		boolean exitMessageProcessor = true;

	    if ((debugLevel & DEBUG_STARTUP) != 0) {
			getLogger().info("Start of processMessages");
	    }

		try {
	        if (serverSocket != null) {
		        if ((debugLevel & DEBUG_STARTUP) != 0) {
		        	getLogger().info("Accepting connection on asynchronous socket");
		        }

				asyncSocket = serverSocket.accept();
		        if ((debugLevel & DEBUG_STARTUP) != 0) {
		        	getLogger().info("Asynchronous socket accepted");
		        }
	        }
	        else {
	        	shutdownServer = true;
	        }
				
		} catch(Exception e) {
			getLogger().error("Exception during asyncSocket initialization: \"" + e.toString() + "\"");
			exitMessageProcessor = true;
        	shutdownServer = true;
			if (asyncSocket != null) {
				try {
					asyncSocket.close();
				} catch (Exception eAsync) {
					getLogger().error("Exception during socket close: \"" + eAsync.toString() + "\"");
				}
			}
		}

		try {
			if (asyncSocket != null ) {
				asyncOut = new PrintWriter(asyncSocket.getOutputStream(), true);
				asyncIn = new BufferedReader(new InputStreamReader(asyncSocket.getInputStream()));
		        if ((debugLevel & DEBUG_STARTUP) != 0) {
		        	getLogger().info("asyncIn/asyncOut buffers created");
		        }
		        
				exitMessageProcessor = false;
			}

		} catch (Exception e) {
			getLogger().error("Exception during buffer creation: \"" + e.toString() + "\"");
			if (asyncOut != null) {
				try {
					asyncOut.close();
				} catch (Exception eAsync) {
					getLogger().error("Exception in asyncOut.close: \"" + eAsync.toString() + "\"");
				}
			}
			if (asyncIn != null) {
				try {
					asyncIn.close();
				} catch (Exception eAsync) {
					getLogger().error("Exception in asyncIn.close: \"" + eAsync.toString() + "\"");
				}
			}
			if (asyncSocket != null) {
				try {
					asyncSocket.close();
				} catch (Exception eAsync) {
					getLogger().error("Exception during socket close: \"" + eAsync.toString() + "\"");
				}
			}

		}
		
		
		while (!exitMessageProcessor) {
			debugLevel = getApplicationData().getProcessData("FRIDebugLevel").getValue();

			// Wait for a command to be received
			try {
				newCommand = null;
				newCommand = asyncIn.readLine();
		        if ((debugLevel & DEBUG_ASYNCMESSAGES) != 0) {
		           	getLogger().info("Command received: \"" + newCommand + "\"");
		        }

			} catch (Exception e) {
				getLogger().error("Exception on readLine: \"" + e.toString() + "\"");
				// readLine exceptions should just shut everything down
				doStopSession();
				exitMessageProcessor = true;
				if (mc != null) {
					mc.cancel();
					lbr.move(shortPositionHold);
				}
				response = "ERROR";
			}
					
			try {
				response = "Eh?";//ProcessCommand(newCommand);
				if (newCommand != null) {
					parsedCommand = newCommand.split(":");
					
					if (newCommand.equals("CONNECT")) {
						response = "CONNECTED";
						}
					if (newCommand.equals("EXIT")) {
						exitMessageProcessor = true;
						response = "EXITING";
						}
					if (newCommand.equals("SHUTDOWN")) {
						shutdownServer = true;
						exitMessageProcessor = true;
						response = "SHUTDOWN";
						}
					
					// Now check for commands with their own subcommand processor
					if (parsedCommand[0].equals("SESSION")) {
						response = doSessionCommand(parsedCommand);
					}
					if (parsedCommand[0].equals("SSR")) {
						response = doSSR(parsedCommand[1]);
						}
					if (parsedCommand[0].equals("COMMANDMODE")) {
						response = createOverlay(parsedCommand[1]);
					}
					if (parsedCommand[0].equals("CONTROLMODE")) {
						response = createOverlayMotion(parsedCommand[1]);
					}
					if (parsedCommand[0].equals("MOTION")) {
						response = doMotionCommand(parsedCommand[1]);
					}

				} // (newCommand != null)
				else {
					// Treat a null newCommand the same as an EXIT command
					 exitMessageProcessor = true;
				}
			} catch(Exception e) {
				getLogger().error("Exception processing command \"" + newCommand + "\": " + e.toString() );
			}

			// Now try to send the response
			try {
				if ((debugLevel & DEBUG_COMMANDS) != 0) {
					getLogger().info("Sending response: \"" + response + "\"");
				}
				asyncOut.println(response);
				asyncOut.flush();
				if ((debugLevel & DEBUG_COMMANDS) != 0) {
					getLogger().info("After flush");
				}
			} catch(Exception e) {
				getLogger().error("Exception sending response \"" + response + "\": " + e.toString());
			}

		}  // while !exitMessageProcessor
				
   	    if ((debugLevel & DEBUG_SHUTDOWN) != 0) {
   	    	getLogger().info("processMessages: Cleaning up asyncSocket");
   	    }
			
		if (asyncSocket != null) {
			try {
				asyncSocket.close();
			} catch (Exception eAsync) {
				getLogger().error("Exception during asyncSocket.close: \"" + eAsync.toString() + "\"");
			}
		}

		if ((debugLevel & DEBUG_SHUTDOWN) != 0) {
   	    	getLogger().info("asyncSocket closed");
   	    }
				
	}


	public void initialize() {
		initSuccess = false;
		kukaController = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(kukaController, "LBR_iiwa_7_R800_1");
		debugLevel = getApplicationData().getProcessData("FRIDebugLevel").getValue();
		
		positionControlMode = new PositionControlMode();
		shortPositionHold = new PositionHold(positionControlMode, 5, TimeUnit.MILLISECONDS);
		
		try {
	        if ((debugLevel & DEBUG_STARTUP) != 0) {
	        	getLogger().info("Creating asynchronous serverSocket");
	        }
			
			serverSocket = new ServerSocket();
	        if ((debugLevel & DEBUG_STARTUP) != 0) {
	        	getLogger().info("Setting reuse address on asynchronous socket");
	        }
			
			serverSocket.setReuseAddress(true);
	        if ((debugLevel & DEBUG_STARTUP) != 0) {
	        	getLogger().info("Creating server socket address on portNumber");
	        }
			
			// Creating the address using the local IP address isn't working; use just the port number
//			InetSocketAddress serverSockAddr = new InetSocketAddress(hostName, portNumber);
			InetSocketAddress serverSockAddr = new InetSocketAddress(portNumber);
	        if ((debugLevel & DEBUG_STARTUP) != 0) {
	        	getLogger().info("Binding asynchronous socket to portNumber");
	        }
			
			serverSocket.bind(serverSockAddr);
			initSuccess = true;
		} catch(Exception e) {
			getLogger().error("Exception during serverSocket initialization: \"" + e.toString() + "\"");
			if (serverSocket != null) {
				try {
					serverSocket.close();
				} catch (Exception eServer) {
					getLogger().error("Exception during serverSocket close: \"" + eServer.toString() + "\"");
				}
			}

		}
	}



@Override
	public void dispose() {
		getLogger().info("dispose() called: why?");

	    if ((mc != null) && !mc.isFinished()) {
	    	mc.cancel();
	    	lbr.move(shortPositionHold);
	    }
	    
	    try {
	    	try {
	    		if (friSession != null) {
		    		if ((debugLevel & DEBUG_CLEANUP) != 0) {
		    			getLogger().info("dispose: Shutting down FRI session");
		    		}
	    	         
	    	        friSession.close();
	    	        friSession = null;
		    		if ((debugLevel & DEBUG_CLEANUP) != 0) {
		    		    getLogger().info("dispose: FRI session closed");
		    		}
	    		}
	    	} catch (Exception e) {
	    		getLogger().error("dispose: Exception closing friSession: \"" + e + "\"");
	    	}

	    	try {
	    		if (asyncSocket != null) {
	    			asyncSocket.close();
	    		}
	    		if ((debugLevel & DEBUG_CLEANUP) != 0) {
	    			getLogger().info("dispose: asyncSocket closed");
	    		}

	    	} catch (IOException ioe) {
	    		getLogger().error("dispose: Exception closing asyncSocket: \"" + ioe + "\"");
	    	}

	    	try {
	    		if (serverSocket != null) {
	    			serverSocket.close();
	    		}
	    		if ((debugLevel & DEBUG_CLEANUP) != 0) {
	    			getLogger().info("dispose: serverSocket closed");
	    		}
	    		 
	    	} catch (IOException ioe) {
	    		getLogger().error("dispose: Exception closing serverSocket: \"" + ioe + "\"");
	    	}
	    	 
	    }
	    finally {
	        super.dispose();
	    }
	}


	public void run() {
		if (!initSuccess) {
			getLogger().info("initialize not successful; abandoning run");
			return;
		}
		
		getLogger().info("Fast Robot Interface server");
		
		while (!shutdownServer) {
			debugLevel = getApplicationData().getProcessData("FRIDebugLevel").getValue();

			processMessages();
			if ((debugLevel & DEBUG_SHUTDOWN) != 0) {
				getLogger().info("processMessages() returned");
			}

			// Clean up resources that might have been left open
			// mc first, then friSession, then asyncSocket
    		if ((mc != null) && !mc.isFinished()) {
    			mc.cancel();
    			lbr.move(shortPositionHold);
    		}
    		if ((mc != null) && !mc.isFinished()) {
    			getLogger().warn("Canceled motion mc, but not reporting isFinished");
    		}
    		while ((mc != null) && !mc.isFinished()) {
    			try {
					ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
						//getLogger().error("Exception on ThreadUtil.milliSleep waiting for motion to start:" + e.toString());
					}
    		}
			if ((debugLevel & DEBUG_SHUTDOWN) != 0) {
    	    	getLogger().info("run: mc now null or isFinished");
    	    }

			
	    	try {
	    		if (friSession != null) {
	    			if ((debugLevel & DEBUG_SHUTDOWN) != 0) {
		        	    	getLogger().info("run: Shutting down FRI session (remote neglected to do so)");
		        	    }
	    	        friSession.close();
	    	        friSession = null;
	    	        if ((debugLevel & DEBUG_SHUTDOWN) != 0) {
	    	        	getLogger().info("run: FRI session closed");
	    	        }
	    		    
	    		}
	    	} catch (Exception e) {
	    		getLogger().error("run: Exception closing friSession: \"" + e + "\"");
	    	}
	    	
			try {
				if (asyncSocket != null) {
					asyncSocket.close();
				}
			} catch (Exception e) {
				getLogger().error("run: Exception closing asyncSocket: \"" + e + "\"");
			}
			


		}  // while (!shutdownServer)
		
		// Last action: close the serverSocket, shutting down completely
		if (serverSocket != null) {
			try {
				serverSocket.close();
			} catch (Exception eServer) {
				getLogger().error("Exception during socket close: \"" + eServer.toString() + "\"");
			}
		}

	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		FRIServer app = new FRIServer();
		app.runApplication();
	}
}
