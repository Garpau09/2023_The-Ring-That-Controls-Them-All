package frc.robot.Objects;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.util.ClawUtils;
import frc.robot.Constants;
import frc.robot.Constants.ClawConstants;

public class Arm {
    private WPI_TalonFX m_armMotorLeft;
    private WPI_TalonFX m_armMotorRight;

    private double targetTheta;

    private double minTheta, maxTheta;

    private DigitalInput resetEncoder = new DigitalInput(0);

    /*
     * Left Motor - Leader
     * Right Motor - Follower - NEVER SET OR READ TO/FROM RIGHT MOTOR, DO ALL
     * CALCULATIONS BASED ON LEFT MOTOR
     */

    /**
     * @param motorPID P, I, and D values of wrist motor
     * @param peakOutput Peak percent output of closed loop control
     * @param maxAcceleration Units per 100ms per second
     * @param maxVelocity Units per 100 ms
     */
    public Arm(double[] motorPID, double peakOutput, double maxAcceleration, double maxVelocity) {
        m_armMotorLeft = new WPI_TalonFX(Constants.ClawConstants.ClawMotorLeftID);
        m_armMotorRight = new WPI_TalonFX(Constants.ClawConstants.ClawMotorRightID);

        m_armMotorLeft.config_kP(0, motorPID[0]);
        m_armMotorLeft.config_kI(0, motorPID[1]);
        m_armMotorLeft.config_kD(0, motorPID[2]);
        m_armMotorLeft.configClosedLoopPeakOutput(0, peakOutput);

        m_armMotorLeft.setInverted(true);

        m_armMotorRight.follow(m_armMotorLeft);
        m_armMotorRight.setInverted(InvertType.OpposeMaster);

        m_armMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        m_armMotorLeft.setNeutralMode(NeutralMode.Brake);
        m_armMotorRight.setNeutralMode(NeutralMode.Brake);

        m_armMotorLeft.configMotionAcceleration(maxAcceleration);
        m_armMotorLeft.configMotionCruiseVelocity(maxVelocity);
    }

    public boolean isResetEncoderPushed(){
        return !resetEncoder.get();
    }

    /**
     * 
     * @param armAngle Position of arm in degrees
     */
    public void setPosition(){
        double armAngleInEncoderUnits = ClawUtils.degreesToEncoderUnits(targetTheta, ClawConstants.armGearRatio);

        
        m_armMotorLeft.set( 
            ControlMode.MotionMagic,
            armAngleInEncoderUnits,
            DemandType.ArbitraryFeedForward,
            ClawConstants.armFeedForward
             * java.lang.Math.sin(Math.toRadians(
                    ClawUtils.encoderUnitsToDegrees(m_armMotorLeft.getSelectedSensorPosition(), ClawConstants.armGearRatio))));
    }

    public double constrain(double theta){
         
        if(theta >= maxTheta){
            return maxTheta;
        } else if (theta <= minTheta){
            return minTheta;
        }

        return theta;
    }

    public void setTargetTheta(double theta){
        theta = constrain(theta);
        targetTheta = theta;
    }

    public void zeroEncoder() {
        m_armMotorLeft.setSelectedSensorPosition(0);    
      }

    public double getRelativeAngle(){
        return ClawUtils.encoderUnitsToDegrees(m_armMotorLeft.getSelectedSensorPosition(), ClawConstants.armGearRatio);
    }

    public double getRawEncoderUnits(){
        return m_armMotorLeft.getSelectedSensorPosition();
    }

    public boolean getAtTarget(double deadBand) {
		// get absolute value of the difference
		double error = Math.abs(getRelativeAngle() - targetTheta);

		if (error < deadBand) {
			return true;
		}
		return false;
	}
}
