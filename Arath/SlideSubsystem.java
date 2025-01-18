import com.arcrobotics.ftclib.command.SubsystemBase;

public class SlideSubsystem extends SubsystemBase {

    //Positional PID for the linear slide
    public static double PIDControl(double reference, double state){
        double error = reference - state;
        BotConstants.integralSum += error * BotConstants.timer.seconds();
        double derivative = (error - BotConstants.lastError) / BotConstants.timer.seconds();
        BotConstants.lastError = error;

        BotConstants.timer.reset();

        double output = (error * BotConstants.Kp) + (derivative * BotConstants.Kd) + (BotConstants.integralSum * BotConstants.Ki);
        return output;
    }

    //Positional PID for the linear slide
    public static double PIDControlH(double reference, double state){
        double error = reference - state;
        BotConstants.integralSumH += error * BotConstants.timerH.seconds();
        double derivative = (error - BotConstants.lastErrorH) / BotConstants.timerH.seconds();
        BotConstants.lastErrorH = error;

        BotConstants.timerH.reset();

        double output = (error * BotConstants.KpH) + (derivative * BotConstants.KdH) + (BotConstants.integralSumH * BotConstants.KiH);
        return output;
    }

    //Finds the elevator power for the slides given a specific count
    public static void findElevatorPower(){
        BotConstants.powerLeft = PIDControl(BotConstants.reference, BotConstants.leftElevator.getCurrentPosition());
        BotConstants.powerRight = PIDControl(BotConstants.reference, BotConstants.rightElevator.getCurrentPosition());
        BotConstants.powerH = PIDControlH(BotConstants.referenceH, BotConstants.horizontalElevator.getCurrentPosition());
    }

    //Sets the elevators power using the values from the PID
    public static void setElevatorPower(double LPower, double RPower, double HPower){
        BotConstants.leftElevator.setPower(LPower);
        BotConstants.rightElevator.setPower(RPower);
        BotConstants.horizontalElevator.setPower(HPower);
    }

    //Sets the counter and makes sure it doesn't go over or under the limit
    public static void setCounter(int sign){
        BotConstants.counter += sign;
        if(BotConstants.counter >= 2) {
            BotConstants.counter = 1;
        }else if(BotConstants.counter <= -1){
            BotConstants.counter = 0;
        }

        BotConstants.reference = BotConstants.references[BotConstants.counter];
        BotConstants.timesSquarePressed = 0;
    }

    public static void setCounterH(int sign){
        BotConstants.counterH += sign;
        if(BotConstants.counterH >= 3) {
            BotConstants.counterH = 2;
        }else if(BotConstants.counterH <= -1) {
            BotConstants.counterH = 0;
            BotConstants.timesSquarePressed = 0;
        }

        BotConstants.referenceH = BotConstants.referencesH[BotConstants.counterH];
    }
}
