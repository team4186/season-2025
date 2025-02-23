//package frc.robot.subsystems;
//import com.revrobotics.RelativeEncoder;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.Command;
//
//import edu.wpi.first.wpilibj.DigitalInput;
//import frc.robot.Constants;
//import frc.robot.sparkmaxconfigs.Components;
//import frc.robot.sparkmaxconfigs.SingleMotor;
//
//
//public class EndEffector extends SubsystemBase {
//    private final SingleMotor endEffectorMotor;
//    //private RelativeEncoder encoder;
//    private DigitalInput luna;
//
//
//    public EndEffector(SingleMotor endEffectorMotor, DigitalInput luna){
//        //encoder = endEffectorMotor.motor.getEncoder();
//        this.luna = luna;
//        this.endEffectorMotor = endEffectorMotor;
//    }
//
//
//    public Command intake(){
//        try {
//            if (!luna.get()) {
//                endEffectorMotor.accept(Constants.EndEffectorConstants.END_EFFECTOR_SPEED);
//            } else {
//                endEffectorMotor.stop();
//            }
//
//        } catch (IllegalStateException e) {
//            endEffectorMotor.stop();
//            String msg = "EndEffector Beambreak error: " + e;
//            System.out.println(msg);
//        }
//
//        return null;
//    }
//
//
//    public Command eject() {
//        endEffectorMotor.accept(-Constants.EndEffectorConstants.END_EFFECTOR_SPEED);
//
//        return null;
//    }
//
//
//    public Command stop(){
//        endEffectorMotor.stop();
//
//        return null;
//    }
//}
