import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class ClawSubsystem extends SubsystemBase {



    public static void setClawPosition(int position) {
        switch(position) {
            case 0:
                // 0 = pick up from clawSub position - used to drop sample into bucket
                BotConstants.claw.setPosition(BotConstants.clawOpenPos);

                BotConstants.clawOpeningPickup = true;
                BotConstants.delayTimer.reset();

                break;

            case 1:
                // 1 = bucket position - gets in position to drop off in bucket
                BotConstants.clawVert.setPosition(BotConstants.clawVertPickUpPos);

                BotConstants.clawLowering = true;
                BotConstants.delayTimer.reset();

                break;

            case 2:
                // 2 = pick up specimen from wall position
                BotConstants.claw.setPosition(BotConstants.clawOpenPos);
                BotConstants.clawVert.setPosition(BotConstants.clawVertBucketPos + 0.2);
                //BotConstants.clawRotate.setPosition(BotConstants.clawRotateSpecimenPos);

                BotConstants.clawRotatingSpecimen = true;
                BotConstants.delayTimer.reset();

                break;

            case 3:
                // 3 = hang specimen position
                BotConstants.claw.setPosition(BotConstants.clawClosedPos);
                //Start delay timer
                BotConstants.clawClosing = true;
                BotConstants.delayTimer.reset();

                break;

        }
    }

    public static void checkTimers() {

        //Warns driver about incoming endgame
        if(BotConstants.matchTimer.seconds() >= 85) {
            BotConstants.matchTimer.reset();
        }

        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<< GOING TO HANG POSITION >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime && BotConstants.clawClosing) {
            BotConstants.reference = BotConstants.referenceHang;

            BotConstants.raising = true;
            BotConstants.delayTimer.reset();
            BotConstants.clawClosing = false;
        }

        //Waits for the slides to raise after grabbing specimen(Above Timer)
        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime * 2 && BotConstants.raising) {
            //Raises the slides to vertical position to flip around
            BotConstants.clawVert.setPosition(BotConstants.clawVertBucketPos + 0.2);

            BotConstants.clawRaisingHang = true;
            BotConstants.delayTimer.reset();
            BotConstants.raising = false;
        }

        //Waits for the claw to raise to hang position after grabbing specimen (Above Timer)
        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime && BotConstants.clawRaisingHang) {
            BotConstants.clawRotate.setPosition(BotConstants.clawRotateHangPos);

            BotConstants.clawRaisingHang = false;
            BotConstants.delayTimer.reset();
            BotConstants.clawRotatingHang = true;
        }


        ////// <<<<<<<<<<<<<<<<<<<, NEW CODE TO FIX INSPECTION >>>>>>>>>>>>>>>>>>>.
        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime * 1.5 && BotConstants.clawRotatingHang) {
            BotConstants.clawVert.setPosition(BotConstants.clawVertHangPos);
            BotConstants.clawRotatingHang = false;
        }


        // <<<<<<<<<<<<<<<<<<<<<<<<<<>>>> FOR GOING TO BUCKET POSITION >>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime && BotConstants.clawLowering) {
            BotConstants.claw.setPosition(BotConstants.clawClosedPos);

            BotConstants.clawClosingBucket = true;
            BotConstants.delayTimer.reset();
            BotConstants.clawLowering = false;
        }

        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime && BotConstants.clawClosingBucket) {
            BotConstants.clawSub.setPosition(BotConstants.clawSubOpenPos);

            BotConstants.clawSubOpening = true;
            BotConstants.delayTimer.reset();
            BotConstants.clawClosingBucket = false;
        }

        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime && BotConstants.clawSubOpening) {
            BotConstants.clawVert.setPosition(BotConstants.clawVertBucketPos);

            BotConstants.clawRaisingBucket = true;
            BotConstants.delayTimer.reset();
            BotConstants.clawSubOpening = false;
        }

        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime * 2 && BotConstants.clawRaisingBucket) {
            BotConstants.clawRotate.setPosition(BotConstants.clawRotateBucketPos);
            BotConstants.clawRaisingBucket = false;
        }

        // <<<<<<<<<<<<<<<<<<<<<<<<<< FOR GOING TO PICKUP POSITION >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime && BotConstants.clawOpeningPickup) { //old was standardTime * 3
            BotConstants.clawRotate.setPosition(BotConstants.clawRotatePickUpPos);

            BotConstants.clawRotatingPickUp = true;
            BotConstants.delayTimer.reset();
            BotConstants.clawOpeningPickup = false;
        }

        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime && BotConstants.clawRotatingPickUp) {
            BotConstants.clawVert.setPosition(BotConstants.clawVertPickUpPos);
            BotConstants.clawRotatingPickUp = false;
        }

        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<< FOR PICKING UP SAMPLE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime && BotConstants.clawSubClosing) {
            BotConstants.clawSubRotate.setPosition(BotConstants.clawSubRotateUp);

            BotConstants.clawSubRotatingUp = true;
            BotConstants.delayTimer.reset();
            BotConstants.clawSubClosing = false;
        }

        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime && BotConstants.clawSubRotatingUp) {
            SlideSubsystem.setCounterH(-2);
            BotConstants.clawSubRotatingUp = false;
        }

        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< FOR EXTENDING H SLIDE >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime * 2 && BotConstants.clawSubRotatingDown) {
            BotConstants.clawSub.setPosition(BotConstants.clawSubOpenPos);
            BotConstants.clawSubRotatingDown = false;
        }

        // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< FOR PICKING UP SPECIMEN FROM WALL >>>>>>>>>>>>>>>>>>>>>
        if(BotConstants.delayTimer.milliseconds() >= BotConstants.standardTime * 2.5 && BotConstants.clawRotatingSpecimen) {
            BotConstants.clawRotate.setPosition(BotConstants.clawRotateSpecimenPos);
            //BotConstants.clawVert.setPosition(BotConstants.clawVertBucketPos + 0.1);
            BotConstants.delayTimer.reset();
            BotConstants.clawLowerIngToSpecimen = true;
            BotConstants.clawRotatingSpecimen = false;
        }

        if(BotConstants.delayTimer.milliseconds() >+ BotConstants.standardTime * 7 && BotConstants.clawLowerIngToSpecimen) {
            BotConstants.clawVert.setPosition(BotConstants.clawVertSpecimenPos);
            BotConstants.clawLowerIngToSpecimen = false;

        }

    }
}
