# Superstructure Graph

```mermaid
%%{init: {"flowchart": {"defaultRenderer": "elk"}} }%%
graph TD;

IDLE

SCORE_CORAL --> IDLE
SPIT_CORAL --> IDLE
SPIT_ALGAE --> IDLE

IDLE --> INTAKE_CORAL
IDLE --> HOME
HOME --> IDLE
INTAKE_CORAL --> IDLE

subgraph CORAL

INTAKE_CORAL --> READY_CORAL
READY_CORAL --> SPIT_CORAL

READY_CORAL --> PRE_L1
READY_CORAL --> PRE_L2
READY_CORAL --> PRE_L3
READY_CORAL --> PRE_L4

SCORE_CORAL
PRE_L1 --> SCORE_CORAL
PRE_L2 --> SCORE_CORAL
PRE_L3 --> SCORE_CORAL
PRE_L4 --> SCORE_CORAL

PRE_L1 --> READY_CORAL
PRE_L2 --> READY_CORAL
PRE_L3 --> READY_CORAL
PRE_L4 --> READY_CORAL

ANTI_JAM

end

IDLE --> INTAKE_ALGAE_LOW
IDLE --> INTAKE_ALGAE_HIGH

INTAKE_ALGAE_LOW --> IDLE
INTAKE_ALGAE_HIGH --> IDLE

SCORE_ALGAE --> IDLE

subgraph ALGAE

INTAKE_ALGAE_LOW --> READY_ALGAE
INTAKE_ALGAE_HIGH --> READY_ALGAE

READY_ALGAE --> SPIT_ALGAE
READY_ALGAE --> PRE_PROCESSOR


PRE_PROCESSOR --> SCORE_ALGAE

PRE_PROCESSOR --> READY_ALGAE

end

IDLE --> ANTI_JAM
ANTI_JAM --> IDLE
READY_CORAL --> ANTI_JAM

IDLE --> PRE_CLIMB
PRE_CLIMB --> IDLE
CLIMB --"?"--> IDLE

subgraph climb [CLIMB]

SPIT_CORAL --> PRE_CLIMB
SPIT_ALGAE --> PRE_CLIMB
PRE_CLIMB --> CLIMB


end
```

## Descriptions

| State | Description | End |
| ----- | ----------- | --- |
| IDLE  | The robot has no game pieces, the elevator and climber is stowed, and the intake is not running.| An intake or climb request is triggered. |
| INTAKE_CORAL | The robot is ready to intake a coral, the elevator is at the correct position and the intake is running.| A stow request or game piece is detected. |
| READY_CORAL | The robot has a coral in the intake. The elevator is retracted. | A scoring, prescoring, or spit request is triggered. |
| SPIT_CORAL | The robot ejects a coral from the intake onto the ground. | No game piece is detected and a timer passes. |
| PRE_{L1-4} | Move the elevator to the corresponding position in preparation for scoring. | The elevator is in the correct position and the score request is triggered. |
| SCORE_CORAL | The robot holds the elevator in it's position and ejects a held coral. | No coral is detected in the robot. |
| ANTI_JAM | The robot extends the intake and elevator to clear the center of the robot to help remove coral that may be stuck. | The corresponding request ends. |
| INTAKE_ALGAE_{Location} | The robot moves the elevator to the relevant position and runs the intake to intake algae. | When the request ends. |
| READY_ALGAE | The robot has an algae in the intake. The elevator is retracted. | A scoring, prescoring, or spit request is triggered. |
| SPIT_ALGAE | The robot ejects an algae from the intake onto the ground. | The corresponding request ends. |
| PRE_PROCESSOR | Move the elevator to the corresponding position in preparation for scoring. | The elevator is in the correct position and the score request is triggered. |
| SCORE_ALGAE | The robot holds the elevator in it's position and ejects a held algae. | The corresponding request ends. |
| PRE_CLIMB | The robot extends the climber and runs any motion required to latch onto the cage. | When the robot grabs the cage and a relevant request is triggered. |
| CLIMB | The robot pivots the climber back up. | A relevant cancel request is run, although this may be dangerous if the robot is hanging. |
| HOME | The robot homes in both the elevator and climber passively | The corresponding request ends. |
