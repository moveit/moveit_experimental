package application;


import java.util.concurrent.TimeUnit;

import com.kuka.common.ThreadInterruptedException;
import com.kuka.common.ThreadUtil;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

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
public class SelectableCompliance extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr;
	MediaFlangeIOGroup mfIO;

	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getRobot(kuka_Sunrise_Cabinet_1, "LBR_iiwa_7_R800_1");
		mfIO = new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
		
		
		
		
	}

	public void run() {
		int selectedMode = 0;
		double jointStiffness;
		IMotionContainer mc;

		JointImpedanceControlMode limp = new JointImpedanceControlMode(lbr.getJointCount());
		
		CartesianImpedanceControlMode complyZ = new CartesianImpedanceControlMode();
		complyZ.parametrize(CartDOF.TRANSL).setStiffness(5000.0);
		complyZ.parametrize(CartDOF.ROT).setStiffness(300.0);
		complyZ.parametrize(CartDOF.Z).setStiffness(0.0);

		CartesianImpedanceControlMode complyXY = new CartesianImpedanceControlMode();
		complyXY.parametrize(CartDOF.TRANSL).setStiffness(5000.0);
		complyXY.parametrize(CartDOF.ROT).setStiffness(300.0);
		complyXY.parametrize(CartDOF.X).setStiffness(0.0);
		complyXY.parametrize(CartDOF.Y).setStiffness(0.0);

		CartesianImpedanceControlMode complyZA = new CartesianImpedanceControlMode();
		complyZA.parametrize(CartDOF.TRANSL).setStiffness(5000.0);
		complyZA.parametrize(CartDOF.ROT).setStiffness(300.0);
		complyZA.parametrize(CartDOF.Z).setStiffness(0.0);
		complyZA.parametrize(CartDOF.A).setStiffness(0.0);
		
		PositionControlMode positionControlMode = new PositionControlMode();

		getApplicationControl().setApplicationOverride(0.1);
		do {
			selectedMode = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Enter demo mode", "PosCtrl", "JointImp", "ComplyZ", "ComplyXY", "ComplyZA", "Exit");
			switch (selectedMode) {
			case 0: // Position control
				while (!mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc = lbr.moveAsync(positionHold(positionControlMode, -1, TimeUnit.SECONDS));
				while (mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc.cancel();
				lbr.move(ptp(lbr.getCurrentJointPosition()));
				break;
			case 1: // Joint impedance control, low stiffness
				jointStiffness = getApplicationData().getProcessData("jointImpedance").getValue();
				limp.setStiffnessForAllJoints(jointStiffness);
				while (!mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc = lbr.moveAsync(positionHold(limp, -1, TimeUnit.SECONDS));
				while (mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc.cancel();
				lbr.move(ptp(lbr.getCurrentJointPosition()));
				break;
			case 2: // Compliant in Z
				while (!mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc = lbr.moveAsync(positionHold(complyZ, -1, TimeUnit.SECONDS));
				while (mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc.cancel();
				lbr.move(ptp(lbr.getCurrentJointPosition()));
				break;
			case 3: // Compliant in XY
				while (!mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc = lbr.moveAsync(positionHold(complyXY, -1, TimeUnit.SECONDS));
				while (mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc.cancel();
				lbr.move(ptp(lbr.getCurrentJointPosition()));
				break;
			case 4: // Compliant in Z and A
				while (!mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc = lbr.moveAsync(positionHold(complyZA, -1, TimeUnit.SECONDS));
				while (mfIO.getUserButton()) {
					try {
						ThreadUtil.milliSleep(100);
					} catch (ThreadInterruptedException e) {
						// Ignore this exception
					}
				}
				mc.cancel();
				lbr.move(ptp(lbr.getCurrentJointPosition()));
				break;
			default:
				selectedMode = -1;
			}
		} while (selectedMode >= 0);
		
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		SelectableCompliance app = new SelectableCompliance();
		app.runApplication();
	}
}
