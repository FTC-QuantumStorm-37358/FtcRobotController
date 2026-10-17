package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Autonomous for Red Team Two.
 * Select this on the Driver Hub when our team is Red Team Two.
 */
@Autonomous(name = "Red Team Two Auto", group = "Quantum Storm", preselectTeleOp = "Quantum Storm Two Driver")
public class RedTeamTwoAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        try {
            AutoStateMachine auto = new AutoStateMachine(this, AutoStartPosition.RED_2);
            waitForStart();
            if (!isStopRequested()) auto.run();
        } catch (java.util.concurrent.CancellationException ignored) {
            // STOP during INIT is a normal exit.
        }
    }
}
