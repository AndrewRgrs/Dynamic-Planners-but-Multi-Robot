# Dynamic path planners used with multiple, non-cooperative robots
Implementation of various path planners for dynamic environments. The 'dynamic' obstacles for this project were in fact other simulated robots using their own instance of the path planner operating within the same environment.
This was written for a graduate course project and thus was never meant to be super readable (sorry). 

Planners implemented include:
- DRRT,
- DRRT*
- RRTX,
- Velocity Obstacles.

The multi-agent experiment scripts for running each planner on 4 separate robots can be found within the 'testing' folder. Also included in that folder is a script to run a single-robot version of a selected planner.

The implementations (classes) of each planner can be found within the 'algorithms' folder.


Example of 4 robots trying to navigate around each other using DRRT* :
![Gif of 4 robots trying to get around each other each using DRRT* planner](https://github.com/AndrewRgrs/Multi-Agent-Planners/assets/77746490/b8169c82-6d30-4778-b216-4a23e8fe6ebb)




