package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Red Team One: shoot preloads, collect Garden pollen with Limelight, park by 26 s.
 * Starting position: red hive facing the robot is tilted up.
 * Select this on the Driver Hub when our team is Red Team One.
 */
@Autonomous(name = "Red Team One Auto", group = "Quantum Storm", preselectTeleOp = "Quantum Storm Two Driver1")
public class RedTeamOneAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        try (AutoStateMachine auto = new AutoStateMachine(this, AutoStartPosition.RED_1)) {
            waitForStart();
            if (!isStopRequested()) auto.run();
        } catch (java.util.concurrent.CancellationException ignored) {
            // STOP during INIT is a normal exit.
        }
    }
}
