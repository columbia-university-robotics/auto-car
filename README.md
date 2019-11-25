# 🤖 auto-car

Mono-repository for CURC's autonomous car code.

## Project Structure
```
src
  - auto
    > control.py: ROS publisher to motor interface for simple controlling
    > test.py: Unit tests for motor control publisher
  - motor
    > interface.py: ROS subcriber interface for motor commands
  - slack
    > notify.py: Sends car's IP address to Slack for convenience
  - util
    > logger.py: Verbose, class-based logger for debugging
```

## Running/Testing
Ensure you have ROS Melodic installed on your machine.
1. `git clone`
2. `catkin_make`
3. `source devel/setup.bash`
4. (in new Terminal window) `roscore`
5. `rostopic pub /project_name filename.py`

### ROS Motor Interface
1. Run `roscore` in a different Terminal window.
2. Ensure you've built the entire project using `catkin_make` above.
3. Run `rostopic pub /motor std_msgs/String "MOTOR_1, MOTOR2, MOTOR_3, MOTOR_4" --once`, where
`MOTOR_#` denotes the speed of the appropriate motor between -255 and 255.

## Contributing
To contribute code to this repository, you must:

1. Ensure your current code does not affect major changes from the most recent master branch. To do this, constantly run `git pull` every time you write new code locally.
2. [Submit a pull request](https://github.com/columbia-university-robotics/mate-rov/pulls) to commit the changes to the repository. You'll require **one approval** from another developer before the code can be merged to the `master` branch.
3. Once approved, your remote branch will be committed to `master`.

## Naming Commits

### Do
Provide short, accurate descriptions of your code changes.

For example:

- `Fixed backwards/forwards mixup bug`
- `Changed tracking tolerance to 0.9`

### Don't
Provide long or cryptic commit messages.

For example:

- `Created a new technique that tracks the presence of baby dolphins in the pool and propels the robot to swim directly to these dolphins`
- `Updated driver code`

## Naming Remote Branches

### `feat-FEATURE_NAME`
Name your remote branch this if you are working on an entirely new feature for the project.

### `fix-FEATURE_NAME-##`
Name your remote branch this if you are fixing an already-existing feature. Feel free to add or ommit numbers at the end of the name to discern your branch from others.

## `refactor-FEATURE_NAME`
Name your remote branch this if you are refactoring something in an already-existing feature, such as changing a variable name. The above applies here.

### `test-FEATURE_NAME`
Name your remote branch this if you are testing an already-existing feature. The above applies here too.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
