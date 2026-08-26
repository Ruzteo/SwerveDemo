package frc.robot;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenDriveModule {
    private int ID; 
    private DCMotorSim driveSim; 
    private TalonFX drive; 
    private SimpleMotorFeedforward driveFF; 
    private ProfiledPIDController drivePID; 
    private SwerveModuleState desiredState; 

    public KrakenDriveModule(int ID){
        this.ID = ID; 

        driveFF = new SimpleMotorFeedforward(Constants.SwerveConstants.kDriveFF_S, Constants.SwerveConstants.kAngleFF_V);
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            Constants.SwerveConstants.kDriveMaxVelocity, Constants.SwerveConstants.kDriveMaxAcceleration);

        drivePID = new ProfiledPIDController(
            Constants.SwerveConstants.kDrivePID_P, Constants.SwerveConstants.kDrivePID_I, Constants.SwerveConstants.kDrivePID_D, constraints);
        
        if(Constants.SwerveConstants.isSim){
            driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 0.025, 1), 
            DCMotor.getKrakenX60(1));
       } 
       else{
        drive = new TalonFX(ID); 
       }
    }

    public void setVelocity(double desiredVelocity){
        if(Constants.SwerveConstants.isSim){

        double currentVelocityRad = driveSim.getAngularVelocityRadPerSec();
        double currentVelocity = currentVelocityRad / Units.inchesToMeters(2);// needs to be the radious of the wheels 
        drivePID.setGoal(desiredVelocity);

        double voltagePID = drivePID.calculate(currentVelocity);
        double voltageFF = driveFF.calculate(drivePID.getGoal().velocity); 
        double voltage = voltageFF + voltagePID; 
        voltage = MathUtil.clamp(voltage,-12, 12);
    
        driveSim.setInputVoltage(voltage);
        driveSim.update(Constants.SwerveConstants.kDt);
        }
        else{
            //TODO 
        } 
    }

    public double getVelocity(){
        if(Constants.SwerveConstants.isSim){
            return driveSim.getAngularVelocityRadPerSec() / 
            Units.inchesToMeters(Constants.SwerveConstants.wheelRadiousInches);
        }
        else{return 0d;}
    }


}
