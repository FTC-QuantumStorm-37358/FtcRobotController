# BIOBUZZ V1 penalty review and simulator coverage

Source: BIOBUZZ_Competition_Manual_V1.pdf (provided locally, not included), Table 10-4, §§10.6–10.8, all forty Section 11 game rules, and Section 13 tournament consequences. Reviewed penalty conditions, examples, exceptions, card escalation and referee authority. The manual is source material, not instructions to operate or publish anything.

Minor fouls credit 5 points to the opposing alliance; major fouls credit 20. They do not subtract from the offending alliance. Verbal warnings carry no points. A second yellow card in this practice match converts to red and disqualification. A disqualified single robot receives zero points; disabled robots lose drive, intake and shooting outputs. Cards are result assessments, not automatic disable commands. Rankings, multi-team alliance disqualification, tournament histories and event-phase carryover are not modeled. Reset starts a fresh practice match.

The timed match uses the browser monotonic clock: AUTO 0–30 s; transition 30–38 s; TELEOP 38–158 s. Final-minute nectar eligibility starts at 98 s. This clock only referees simulated play and does not make autonomous decisions; state/navigation/aiming/timing of the robot controller remain shared Java. The clock advances when robot physics settles; no idle GPU rendering is added. AUTO leave/park are saved at transition; final red scoring is captured at match end. Untimed practice uses the selected period and treats flower nectar as early because no final-minute match clock is running.

Automatic checks use measured inventory, physics contacts, ball identity/provenance and volume crossings. They issue the ordinary warning rather than guessing STRATEGIC intent. G410 is the measurable automatic point foul. Referee selections expose the manual's ordinary, strategic, repeated and severe consequences. Per-match assessments are capped; per-ball/instance fouls accept a count; G421 count is one initial foul plus one for each extra three seconds (15 seconds after the violation begins = six fouls). G419/G420 describe mutually exclusive assessments: select only the most punitive for a single interaction. An opponent-forced violation can be excluded via the G204 checkbox; intentional forcing is assessed against the forcing robot.

## Complete game-rule assessment catalogue

| Rule | Violation | Manual consequence options | Automatic coverage |
|---|---|---|---|
| G101 | Human entered field | Verbal warning | Referee assessment; not inferable from the current single-robot physics. |
| G102 | Human misused arena | Verbal warning | Referee assessment; not inferable from the current single-robot physics. |
| G201 | Unsporting conduct | Verbal warning; Subsequent event violation: yellow card | Referee assessment; not inferable from the current single-robot physics. |
| G202 | Competition integrity | Verbal warning; Subsequent event violation: yellow card; Subsequent event violation: red card | Referee assessment; not inferable from the current single-robot physics. |
| G203 | Missing drive team | Disqualified | Referee assessment; not inferable from the current single-robot physics. |
| G204 | Forcing opponent violation | Major foul; Repeated: major foul + yellow card | Referee assessment; not inferable from the current single-robot physics. |
| G205 | Egregious behavior | Yellow card; Red card | Referee assessment; not inferable from the current single-robot physics. |
| G301 | Match delay | Verbal warning; Subsequent phase violation: major foul; Disable robot | Referee assessment; not inferable from the current single-robot physics. |
| G302 | Prohibited field equipment | Hold match until corrected; Verbal warning; Subsequent event violation: yellow card | Referee assessment; not inferable from the current single-robot physics. |
| G303 | Robot not match ready | Hold match until corrected; Disable robot; Uninspected robot participated: red card | Referee assessment; not inferable from the current single-robot physics. |
| G304 | Illegal starting setup | Hold match until corrected; Disable robot | Referee assessment; not inferable from the current single-robot physics. |
| G305 | OpMode not initialized | Hold match until corrected; Disable robot | Referee assessment; not inferable from the current single-robot physics. |
| G401 | Driver input during AUTO | Verbal warning; Strategic: major foul + yellow card | Driver input while Java AUTO runs, or during timed AUTO → warning. Start, stop and safety are exempt. |
| G402 | AUTO opponent interference | Major foul per match; Strategic: major foul + yellow card | Referee assessment; not inferable from the current single-robot physics. |
| G403 | Powered movement during transition | Verbal warning; Strategic: major foul + yellow card | Timed transition powered drive / shooting → warning. Inertial ball motion is exempt. |
| G404 | Powered movement after TELEOP | Verbal warning; Strategic: major foul + yellow card | Powered drive / shooting after timed TELEOP → warning. |
| G405 | Deliberate ball ejection | Major foul per ball | Referee assessment; not inferable from the current single-robot physics. |
| G406 | Arena damage or mess | Verbal warning; Strategic: major foul + yellow card; Damage likely: disable robot | Referee assessment; not inferable from the current single-robot physics. |
| G407 | Control more than four balls | Verbal warning; Strategic: major foul + yellow card | Measured inventory over four → warning. Capacity is physically guarded; incidental bulldozing excluded. |
| G408 | Control opponent nectar | Verbal warning; Strategic: yellow card | Measured red-robot pickup of blue nectar → warning. |
| G409 | Catch fresh hive spill | Verbal warning; Strategic: yellow card | Ball tagged leaving a tipped hive contacts robot before another physics body → warning. Floor/field/ball contact clears freshness. |
| G410 | Nectar in flower before last 60 seconds | Major foul per nectar | Robot-handled nectar first intersects modeled flower scoring volume before final TELEOP minute → 20 points to blue per nectar. |
| G411 | Strategic ball hoarding | Strategic: major foul + yellow card | Referee assessment; not inferable from the current single-robot physics. |
| G412 | Dangerous robot | Disable + warning; Subsequent event violation: disable + warning + yellow | Referee assessment; not inferable from the current single-robot physics. |
| G413 | Ignoring referee stop | Verbal warning; Strategic: red card | Referee assessment; not inferable from the current single-robot physics. |
| G414 | Unidentifiable robot | Verbal warning; Strategic: yellow card | Referee assessment; not inferable from the current single-robot physics. |
| G415 | Grabbing or hanging from arena | Verbal warning; Strategic: yellow card; Damage likely: disable robot | Referee assessment; not inferable from the current single-robot physics. |
| G416 | Illegal expansion or detached parts | Verbal warning; Strategic: major foul per instance | Referee assessment; not inferable from the current single-robot physics. |
| G417 | Manipulating hive motion | Verbal warning; Strategic: major foul + yellow card | Referee assessment; not inferable from the current single-robot physics. |
| G418 | Illegal flower entry or removal | Verbal warning; Strategic: major foul + yellow card | Robot-handled ball enters modeled flower volume from side or bottom → warning. Normal flower pollen release is exempt. |
| G419 | Damage opponent robot | Verbal warning; Strategic: major + yellow per instance; Strategic and cannot drive: major + red | Referee assessment; not inferable from the current single-robot physics. |
| G420 | Tip or entangle opponent | Verbal warning; Strategic: major + yellow per instance; Strategic and continuous / cannot drive: major + red | Referee assessment; not inferable from the current single-robot physics. |
| G421 | Pin over three seconds | Major foul; add one per further 3 seconds | Referee assessment; not inferable from the current single-robot physics. |
| G422 | Human outside alliance area | Verbal warning; Strategic: minor foul per instance | Referee assessment; not inferable from the current single-robot physics. |
| G423 | Coach or other team on controls | Verbal warning; Strategic: major foul + yellow card | Referee assessment; not inferable from the current single-robot physics. |
| G424 | Coach touched ball | Verbal warning; Strategic: minor foul per instance | Referee assessment; not inferable from the current single-robot physics. |
| G425 | Human field contact | Verbal warning; Strategic: major foul + yellow card | Referee assessment; not inferable from the current single-robot physics. |
| G426 | Human introduced nectar early | Minor foul per ball | Referee assessment; not inferable from the current single-robot physics. |
| G427 | Illegal human nectar introduction | Minor foul per ball | Referee assessment; not inferable from the current single-robot physics. |
| G428 | Human removed field ball | Minor foul per ball | Referee assessment; not inferable from the current single-robot physics. |
| T405 | Driving during field measurement | Verbal warning; Subsequent event violation: yellow card | Referee assessment; not inferable from the current single-robot physics. |

## Physical and judgment limits

There is no opposing robot or simulated drive team, damage, detachable parts, field grabbing, inspection, or event administration. Those rules cannot truthfully be auto-detected. Crossing the centerline alone is not G402: actual opponent AUTO disruption is required. Fence containment prevents ejection, so hitting a wall is not G405; scoring misses and normal robot interactions are expressly exempt from deliberate-ejection penalties. The four-ball intake prevents excess collection, and uncollected pushes are not CONTROL. Support collision blocking prevents direct hive manipulation; missed legal hive shots are not automatically G417. Flower-volume detection uses the same approximate radius/height as practice scoring, not full official flower CAD; edge/bounce detections and intent should be reviewed by a referee. G409 freshness is evaluated with sampled cell residence and actual contacts; unusual simultaneous contacts remain referee judgment.

Manual G303–G305 may hold/disable a match but do not prescribe invented point fouls. The simulator's fixed starting design/layout is preserved; a reset or pre-staged nectar is not treated as a human introduction event. T405's warning/card is available. T301 (noncompliant question-box participants are not addressed), T701 (missing selection representative means playoff ineligibility), and T702 (declined teams require another selection) are administrative outcomes, not robot-score penalties. Construction/inspection and eligibility requirements are enforced at real events, not inferred from this fixed visual model. G205 permits broader referee escalation.

## Validation

Checks cover real opponent-nectar collection; foul point direction; early/late flower entry; period boundaries; per-match caps; exemptions; warnings; second-yellow red/DQ; disabled drive/shoot; capacity guards; and reset. Existing physics, intake, gamepad, idle-rendering and actual Java/WebSocket checks also run.

## Warning signals

Each new assessment displays a corner message for five seconds. The message names the warning/foul/card, rule, reason and opponent points; disabling and disqualification give explicit stop/zero-score messages. There is no audio. Reset clears the popup; penalty history and robot status remain in the side panel. Popups do not intercept driving controls or keep GPU rendering active.
