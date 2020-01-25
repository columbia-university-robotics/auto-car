# 🤖 auto-car

Mono-repository for CURC's autonomous car code.

## Starting the Autonomous Car
1. Plug the power cable into the Raspberry Pi.
2. When the car boots up, it should send a message to the `autonomy-bot` channel in the [Slack Workspace](columbiaurobotics.slack.com) containing code to ssh into it (ex. "Hi! SSH into me with `ssh bot@160.39.233.170`.") Copy the code.
3. Open up a new terminal window and paste the code from (2) to boot into the car. When prompted for a password, use `curc`.
4. Run `/home/bot/Desktop/startup.sh` to start up the motor subscriber.
5. In a new Terminal window, repeat steps 2-3 to open up a new instance of the ssh. You can close the tab from part (4) as long as you leave the motor subscriber running (it should repeatedly say something like 'Interface [00:50:17]: [0, 0, 0, 0]').
6. To send a move command to the motors, run `rostopic pub /motor std_msgs/String "0, 0, 0, 0" --once`. The 0's can be replaced by any number between 0 and 255.

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
