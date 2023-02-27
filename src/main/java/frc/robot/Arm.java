package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.util.ClawUtils;
import frc.robot.Constants.Clawstants;

public class Arm {
    private WPI_TalonFX m_armMotorLeft;
    private WPI_TalonFX m_armMotorRight;

    private double armAngleInEncoderUnits = 0;

    private static Arm singleton;
    private DigitalInput resetEncoder = new DigitalInput(0);

    /*
     * Left Motor - Leader
     * Right Motor - Follower - NEVER SET OR READ TO/FROM RIGHT MOTOR, DO ALL
     * CALCULATIONS BASED ON LEFT MOTOR
     */

    private Arm() {
        m_armMotorLeft = new WPI_TalonFX(Constants.Clawstants.ClawMotorLeftID);
        m_armMotorRight = new WPI_TalonFX(Constants.Clawstants.ClawMotorRightID);

        m_armMotorLeft.config_kP(0, 0.5);
        m_armMotorLeft.config_kI(0, 0);
        m_armMotorLeft.config_kD(0, 0);
        m_armMotorLeft.configClosedLoopPeakOutput(0, .4);

        m_armMotorLeft.setInverted(true);

        m_armMotorRight.follow(m_armMotorLeft);
        m_armMotorRight.setInverted(InvertType.OpposeMaster);

        m_armMotorLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        m_armMotorLeft.setNeutralMode(NeutralMode.Brake);
        m_armMotorRight.setNeutralMode(NeutralMode.Brake);

        m_armMotorLeft.configMotionAcceleration(6000);
        m_armMotorLeft.configMotionCruiseVelocity(7000);
    }

    //Since we only have 1 arm, there is never a reason to have multiple arm objects. Thus, we use a singleton.
    public static Arm getInstance(){
        if (singleton == null){
            singleton = new Arm();
        }
        
        return singleton;
    }

    public boolean isResetEncoderPushed(){
        return !resetEncoder.get();
    }

    public void setAngle(double armAngle){
        armAngleInEncoderUnits = ClawUtils.degreesToEncoderUnits(armAngle, Clawstants.armGearRatio);

        
        m_armMotorLeft.set( 
            ControlMode.MotionMagic,
            armAngleInEncoderUnits,
            DemandType.ArbitraryFeedForward,
            Clawstants.armFeedForward * java.lang.Math
                .cos(Math.toRadians(ClawUtils.encoderUnitsToDegrees(m_armMotorLeft.getSelectedSensorPosition(), Constants.Clawstants.armGearRatio))));

    }

    public void zeroEncoder() {
        m_armMotorLeft.setSelectedSensorPosition(0);    
      }

    public void drive(double percent){
        m_armMotorLeft.set(ControlMode.PercentOutput, percent);
    }

    public double getAngle(){
        return ClawUtils.encoderUnitsToDegrees(m_armMotorLeft.getSelectedSensorPosition(), Clawstants.armGearRatio);
    }

    public double getEncoderUnits(){
        return armAngleInEncoderUnits;
    }

    public double getRawEncoderUnits(){
        return m_armMotorLeft.getSelectedSensorPosition();
    }
}
