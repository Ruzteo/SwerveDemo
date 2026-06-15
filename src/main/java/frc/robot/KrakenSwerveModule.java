package frc.robot;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class KrakenSwerveModule {
    SwerveModuleState state; 
    Translation2d position; 

    TalonFX driveMotor; 
    TalonFX angleMotor; 
    AbsoluteEncoder angleEncoder; 

    public KrakenSwerveModule(Translation2d position, int driveMotorID, int angleMotorID, int encoderID){
        state = new SwerveModuleState();
        this.position = position; 
        driveMotor = new TalonFX(driveMotorID);
        angleMotor = new TalonFX(angleMotorID);
    }

    public void updateState(SwerveModuleState newState){
        state = newState; 
    }

    public SwerveModuleState getState(){
        return state; 
    }

    public Translation2d getPosition(){
        return position; 
    }

    
    
}


