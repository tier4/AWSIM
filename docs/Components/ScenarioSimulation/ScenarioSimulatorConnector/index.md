# Scenario Simulator Connector

## Introduction
`ScenarioSimulatorConnector` is a component that allows connecting AWSIM via ZMQ interfaces.

### Prefab
Prefab can be found under the following path:

```
AWSIM/Assets/ScenarioSimulatorConnector/ScenarioSimulatorConnector.prefab
```

### ScenarioSimulatorConnector Components

It contains two scripts:
- ScenarioSimulatorConnector script
- ScenarioSimulatorRequestProcessor script

![ScenarioSimulatorConnectorPrefab](ScenarioSimulatorConnectorPrefab.png)

## ScenarioSimulatorConnector (script)

*ScenarioSimulatorConnector* (script) initializes and maintains the connection via ZeroMQ interface. It processes received bytes and pass it through to ScenarioSimulatorRequestProcessor script.

![ScenarioSimulatorComponentScript](ScenarioSimulatorComponentScript.png)

#### Elements configurable from the editor level
- `Server Response Address` - address on which ZeroMQ interface listens for incoming requests; format: `tcp://<ip>:<port>; it should match the port configuration in launched scenario_simulator_v2
 
## ScenarioSimulatorRequestProcessor (script) 

TODO

### Elements configurable from the editor level

TODO
