# DelegationModule

This is the DelegationModule, a ROS-package for decentralized delegations in multi-agent-systems/multi-robot-systems. 
For these delegations a variation of a first-price sealed-bid auction will be used to ensure a favourable allocation.

## How to use this package?

* Implement the abstract classes [*GoalWrapperBase*](src/delegation_components/goal_wrappers.py) and
    [*CostEvaluatorBase*](src/delegation_components/cost_evaluators.py) according to your needs

* Implement the [*DelegationClientBase*](src/delegation_components/delegation_clients.py) according to your needs
    and instantiate the Client at the 
    components that should be able to participate or start auctions

* Instantiate the [*DelegationManager*](src/delegation_components/delegation_manager.py) (DM) at 
    all participating components and at least one for components
    that should start auctions
   
* Register the corresponding DMs at all components (or automatize the DM construction and registering)

* add the suiting CostEvaluators for the participating DMs

* start delegations as you please

* the DMs should receive a step (*do_step* function)repeatedly to work properly (this can be made directly or via the clients)

For more details see code documentation in form of doc-strings.

## Configuration

This package can be configured using the *dynamic_reconfigure-package* and the [config-file](cfg/DelegationManager.cfg).
For documentation of how to use the dynamic_reconfigure-package see [ROS-Wiki](http://wiki.ros.org/dynamic_reconfigure).
Half of the parameters are for timeout-duration etc. (configure to environment specifications) and the
other half are parameters of the default cost-function (configure these to your needs, if you use the default function).

## Origin of this package

Author: Vito Mengers

This package was created for my bachelor-thesis with the topic (german)
*"Automatisierte Zielzerlegung für Multi-Roboter-Systeme innerhalb eines hybriden Planungssystems"*
(*translated: "Automated task decomposition within a hybrid behaviour network architecture of multi-robot-systems"*).
It is used there inside an expansion of the *ROS Hybrid Behaviour Planner* (RHBP) 
(see on [github](https://github.com/DAInamite/rhbp)).
 