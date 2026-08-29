package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class JoysticDriveCommand extends Command{
    private SwerveDrive drive; 

    public JoysticDriveCommand(SwerveDrive drive){
        addRequirements(drive);
        this.drive = drive; 
    }


    @Override
    public void execute(){
        drive.drive();
    }
}
