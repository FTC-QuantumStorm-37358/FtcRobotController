package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Red Team Two: opposite-wall shots gated by fresh raised own-Hive far-Cell tags.
 * Select this on the Driver Hub when our team is Red Team Two.
 */
@Disabled
@Autonomous(name = "Red Team Two Auto", group = "Quantum Storm", preselectTeleOp = "Quantum Storm Two Driver1")
public class RedTeamTwoAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        try (AutoStateMachine auto = new AutoStateMachine(this, AutoStartPosition.RED_2)) {
            waitForStart();
            if (!isStopRequested()) auto.run();
        } catch (java.util.concurrent.CancellationException ignored) {
            // STOP during INIT is a normal exit.
        }
    }
}
