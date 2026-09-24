package org.firstinspires.ftc.teamcode.quantumstorm;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class StateMachine extends LinearOpMode {

    enum State {
        INIT,
        MOVING,
        READY_TO_SHOOT,
        READY_TO_PICKUP,
        ERROR,
        PARK,
    }

    State state = State.INIT;

    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            switch (state) {
                case INIT:
                    //switch states when something is true

                        state = State.MOVING;

                    break;

                case MOVING:
                    //switch states when something is true

                        state = State.READY_TO_SHOOT;

                    break;

                case READY_TO_SHOOT:
                    //switch states when something is true

                        state = State.READY_TO_PICKUP;

                    break;

                case READY_TO_PICKUP:
                    break;

                case ERROR:
                    break;

                case PARK:
                    break;
            }
        }
    }
}
