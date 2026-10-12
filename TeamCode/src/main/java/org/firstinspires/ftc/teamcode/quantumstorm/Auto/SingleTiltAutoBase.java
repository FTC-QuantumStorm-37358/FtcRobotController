package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.biobuzz.AutoStateMachine.Routine;

/** Opening four-ball attempt, then park; all decisions stay in the shared controller. */
public abstract class SingleTiltAutoBase extends LinearOpMode {
    protected abstract AutoStartPosition startPosition();
    @Override public final void runOpMode(){
        try{
            AutoStateMachine auto=new AutoStateMachine(this,startPosition(),Routine.SINGLE_TILT);
            waitForStart();
            if(!isStopRequested())auto.run();
        }catch(java.util.concurrent.CancellationException ignored){/* STOP during INIT is normal. */}
    }
}
