package frc.robot.Objects;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import frc.lib.util.ClawUtils;
import frc.robot.Constants.ClawConstants;

public class Wrist {
    private WPI_TalonFX m_wristMotor;
    private CANCoder wristAbsolute;

    private double targetTheta;
    
    private double minTheta, maxTheta;

    /**
     * 
     * @param motorPID P, I, and D values of wrist motor
     * @param peakOutput Peak percent output of closed loop control
     * @param maxAcceleration Units per 100ms per second
     * @param maxVelocity Units per 100 ms
     */
    public Wrist(double[] motorPID, double peakOutput, double maxAcceleration, double maxVelocity) {
        m_wristMotor = new WPI_TalonFX(ClawConstants.ClawMotorWristID);
        wristAbsolute = new CANCoder(ClawConstants.wristEncoder);

        m_wristMotor.setInverted(false);
        m_wristMotor.setNeutralMode(NeutralMode.Brake);
        m_wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        m_wristMotor.config_kP(0, motorPID[0]);
        m_wristMotor.config_kI(0, motorPID[1]);
        m_wristMotor.config_kD(0, motorPID[2]);
        m_wristMotor.configClosedLoopPeakOutput(0, peakOutput);
        m_wristMotor.configMotionAcceleration(maxAcceleration);
        m_wristMotor.configMotionCruiseVelocity(maxVelocity);

        wristAbsolute.configFactoryDefault();
        wristAbsolute.configSensorDirection(true);
        wristAbsolute.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        wristAbsolute.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        wristAbsolute.configMagnetOffset(0); //TODO: set angle offset

        /**Initialize built in sensor to match CANcoder absolute encoder */
        m_wristMotor.setSelectedSensorPosition(ClawUtils.degreesToEncoderUnits(wristAbsolute.getAbsolutePosition(), 100));

    }

    public void setPosition(double wristAngle){
        m_wristMotor.set(
         ControlMode.MotionMagic,
         ClawUtils.degreesToEncoderUnits(wristAngle, 100));
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
        targetTheta = Math.toRadians(theta);
    }

    public void zeroRelativeEncoder() {
        m_wristMotor.setSelectedSensorPosition(0);
      }

    public double getRelativeAngle(){
        return ClawUtils.encoderUnitsToDegrees(m_wristMotor.getSelectedSensorPosition(), 100);
    }

    public double getAbsoluteAngle(){
        return wristAbsolute.getAbsolutePosition();
    }

    public void syncEncoders(){
        m_wristMotor.setSelectedSensorPosition(ClawUtils.degreesToEncoderUnits(wristAbsolute.getAbsolutePosition(), 100));
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
