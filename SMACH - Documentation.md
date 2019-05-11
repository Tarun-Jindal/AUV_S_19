### Basics
A module is a file consisting of Python code. A module can define functions, classes and variables.
##### Concept of Inheritence
Execution of a derived class definition proceeds the same as for a base class. When the class object is constructed, the base class is remembered. This is used for resolving attribute references: **if a requested attribute is not found in the class, the search proceeds to look in the base class. This rule is applied recursively if the base class itself is derived from some other class.**<br />
``` class DerivedClassName(BaseClassName)```<br />
```class DerivedClassName(modname.BaseClassName)```

# SMACH (State Machine)
Smach, which stands for "State Machine", is a powerful and scalable Python-based library .<br/>
The image below shows an example state machine used to coordinate actionlib actions that allow the PR2 robot to charge itself at a standard outlet.<br />
To create a Smach state machine, you first create a number of states, and then add those states to a State Machine container.
#### Creating a State
To create a state, you simply inherit from the State base class, and implement the State.execute(userdata) method:

```python
  class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2']):
       # Your state initialization goes here

     def execute(self, userdata):
        # Your state execution goes here
        if xxxx:
            return 'outcome1'
        else:
            return 'outcome2'
```
#### Adding the States to State Machine
```python
sm = smach.StateMachine(outcomes=['outcome4','outcome5'])
  with sm:
     smach.StateMachine.add('FOO', Foo(),
                            transitions={'outcome1':'BAR',
                                         'outcome2':'outcome4'})
     smach.StateMachine.add('BAR', Bar(),
                            transitions={'outcome2':'FOO'})
```
The resulting State Machine looks like: 
![](http://wiki.ros.org/smach/Tutorials/Getting%20Started?action=AttachFile&do=get&target=simple.png)
 - The red boxes show the possible outcomes of the state machine container: outcome4 and outcome

Complete running program : [CODE LINK](https://github.com/rhaschke/executive_smach_tutorials/blob/indigo-devel/examples/state_machine_simple.py)

## Passing User Data between States
A state could require some input data to do its work, and/or it could have some output data it wants to provide to other states. The input and output data of a state is called userdata of the state. While constructing a state, name of the userdata fields it needs/provides can be specified.

### Specifying User Data
```python
 class Foo(smach.State):
     def __init__(self, outcomes=['outcome1', 'outcome2'],
                        input_keys=['foo_input'],
                        output_keys=['foo_output'])

     def execute(self, userdata):
        # Do something with userdata
        if userdata.foo_input == 1:
            return 'outcome1'
        else:
            userdata.foo_output = 3
            return 'outcome2'
```
 - The input_keys list enumerates all the inputs that a state needs to run. A state declares that it expect these fields to exist in the userdata. The execute method is provided a copy of the userdata struct. The state can read from all userdata fields that it enumerates in the input_keys list, but it can't write to any of these fields.

 - The output_keys list enumerates all the outputs that a state provides. The state can write to all fields in the userdata struct that are enumerated in the output_keys list.
### Connecting User Data
When adding states to a state machine, you also need to connect the user data fields, to allow states to pass data to each other. For example, if state FOO produces 'foo_output', and state BAR needs 'bar_input', then you can attach these two user data ports together using name remapping:
```python
sm_top = smach.StateMachine(outcomes=['outcome4','outcome5'],
                          input_keys=['sm_input'],
                          output_keys=['sm_output'])
  with sm_top:
     smach.StateMachine.add('FOO', Foo(),
                            transitions={'outcome1':'BAR',
                                         'outcome2':'outcome4'},
                            remapping={'foo_input':'sm_input',
                                       'foo_output':'sm_data'})
     smach.StateMachine.add('BAR', Bar(),
                            transitions={'outcome2':'FOO'},
                            remapping={'bar_input':'sm_data',
                                       'bar_output1':'sm_output'})
```
The remapping field maps the in/output_key of a state to a userdata field of the state machine.<br />
Runnable example : [Code Link](https://raw.githubusercontent.com/eacousineau/executive_smach_tutorials/hydro-devel/smach_tutorials/examples/user_data2.py)

## Create a Hierarchical State Machine
Here you learn how to nest different state machines, creating a hierarchical state machine.

 - Creating some states :  we create a number of states, each with a number of outcomes, input keys and output keys specified.
 - Creating a hierarchical state machine : We create a top level state machine, and start adding states to it. One of the states we add is another state machine.

The result looks like this. The only point to take away from this is that every state machine is also a normal state. So you can add a state machine to another state machine in the same way you add a state to a state machine. So dealing with userdata is not any different when you deal with hierarchical state machines: the sub state machine specifies input and output keys, and they get remapped when you add the sub state machine to the top level state machine.
![](http://wiki.ros.org/smach/Tutorials/Create%20a%20hierarchical%20state%20machine?action=AttachFile&do=get&target=sm_expanded.png) <br />
Runnable example : [Code Link](https://raw.githubusercontent.com/eacousineau/executive_smach_tutorials/hydro-devel/smach_tutorials/examples/state_machine_nesting2.py)
## Calling Actions from a State Machine:
Refer [Tutorial](http://wiki.ros.org/smach/Tutorials/Calling%20Actions) and [Code](https://raw.githubusercontent.com/eacousineau/executive_smach_tutorials/hydro-devel/smach_tutorials/examples/actionlib2_test.py) .

## Viewing State Machines 
Works using smach viewer. smach viewer is not available in ROS Kinetic. Refer [Tutorial](http://wiki.ros.org/smach/Tutorials/Smach%20Viewer) for reference. 

## Concurrent State Machine
An example of running two states in parallel.<br />
![](http://wiki.ros.org/smach/Tutorials/Concurrent%20States?action=AttachFile&do=get&target=concurrence2.png)
<br />Refer this [Code](https://raw.githubusercontent.com/eacousineau/executive_smach_tutorials/hydro-devel/smach_tutorials/examples/concurrence2.py) for reference.

## State Machine Containers
```add(label, state, transitions=None, remapping=None) ```
## Concurrence container
#### Concurrence Outcome Map
The outcome map of a SMACH concurrence specifies the policy for determining the outcome of the concurrence based on the outcomes of its children. Once all the states in the concurrence have terminated, if one of these child-outcome mappings is satisfied, the concurrence will return its associated outcome. If none of the mappings are satisfied, the concurrence will return its default outcome.
```python
cc = Concurrence(outcomes = ['outcome1', 'outcome2'],
                 default_outcome = 'outcome1',
                 input_keys = ['sm_input'],
                 output_keys = ['sm_output'],
                 outcome_map = {'succeeded':{'FOO':'succeeded',
                                             'BAR':'outcome2'},
                                'outcome3':{'FOO':'outcome2'}})
with cc:
    Concurrence.add('FOO', Foo())
    Concurrence.add('BAR', Bar())
```
The example above specifies the following policy:

 - When 'FOO' has outcome 'succeeded' and 'BAR' has outcome 'outcome2', the state machine will exit with outcome 'succeeded'.
 - When 'FOO' has outcome 'outcome2', the state machine will exit with outcome 'outcome3', independent of the outcome of state BAR.

**OR** we can use callback method:
```python
# gets called when ANY child state terminates
def child_term_cb(outcome_map):

  # terminate all running states if FOO finished with outcome 'outcome3'
  if outcome_map['FOO'] == 'outcome3':
    return True

  # terminate all running states if BAR finished
  if outcome_map['BAR']:
    return True

  # in all other case, just keep running, don't terminate anything
  return False


# gets called when ALL child states are terminated
def out_cb(outcome_map):
   if outcome_map['FOO'] == 'succeeded':
      return 'outcome1'
   else:
      return 'outcome2'


# creating the concurrence state machine
sm = Concurrence(outcomes=['outcome1', 'outcome2'],
                 default_outcome='outcome1',
                 input_keys=['sm_input'],
                 output_keys=['sm_output'],
                 child_termination_cb = child_term_cb,
                 outcome_cb = out_cb)
                 
with sm:
   Concurrence.add('FOO', Foo(),
                   remapping={'foo_in':'input'})

   Concurrence.add('BAR', Bar(),
                   remapping={'bar_out':'bar_out'})
```
 - The child_termination_cb is called every time one of the child states terminates. In the callback function you can decide if the state machine should keep running (return False), or if it should preempt all remaining running states (return True).
 - The outcome_cb is called once when the last child state terminates. This callback returns the outcome of the concurrence state machine.

## Sequence Container
The Sequence container is a StateMachine container, extended with auto-generated transitions that create a sequence of states from the order in which said states are added to the container. <br />
A container Sequence has its outcomes, specified on construction, along with the 'connector_outcome' which is used for the automatic transitions. The constructor signature is:<br />
```__init__(self, outcomes, connector_outcome):```
#### Adding State
 - Adding states to a sequence is the same as to a container.
 - But, each state added will receive an additional transition from it to the state which is added after it. The transition will follow the outcome specified at construction of this container.
 - If one of the transitions given in the dictionary mapping parameter to 'Sequence.add()' follows the connector outcome specified in the constructor, the provided transition will override the automatically generated connector transition.

Example:
```python
sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        connector_outcome = 'succeeded')
with sq:
    Sequence.add('MOVE_ARM_GRAB_PRE', MoveVerticalGripperPoseActionState())
    Sequence.add('MOVE_GRIPPER_OPEN', MoveGripperState(GRIPPER_MAX_WIDTH))
    Sequence.add('MOVE_ARM_GRAB',     MoveVerticalGripperPoseActionState())
    Sequence.add('MOVE_GRIPPER_CLOSE', MoveGripperState(grab_width))
    Sequence.add('MOVE_ARM_GRAB_POST', MoveVerticalGripperPoseActionState())
```
## Iterator Container 
Refer [Tutorial](http://wiki.ros.org/smach/Tutorials/Iterator%20container).
## Wrapping a Container With actionlib
Refer [Tutorial](http://wiki.ros.org/smach/Tutorials/Wrapping%20a%20SMACH%20Container%20With%20actionlib)
