package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = (0.0029945380023522015 + 0.002998626433079991 + 0.003002573993435301) /3 ;
        ThreeWheelConstants.strafeTicksToInches = (0.0029499683853480324 + 0.0029499683853480324 + 0.0029626294358684633) / 3;
        ThreeWheelConstants.turnTicksToInches = (0.0028070177323390676 );
        ThreeWheelConstants.leftY = 6.5;
        ThreeWheelConstants.rightY = -6.5;
        ThreeWheelConstants.strafeX = 5.1965;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "leftBack";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




