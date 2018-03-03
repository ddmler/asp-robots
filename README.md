# asp-robots

This encodes a solution to a planning problem for automated warehouse robots that fetch and carry shelves with products to fulfill given orders. It includes a distributed master/worker solution as well as multiple monolithic encodings with different grades of relaxations for performance tests.

Given a layout of the warehouse and a list of orders to fulfill, this will plan a series of actions for each robot. In the monolithic encoding everything is planned by a single entity whereas for the distributed encoding each robot can plan it's way through the warehouse by itself and asks the master for approval of it's plan.

You can use the [clingo solver](https://potassco.github.io/) to solve an instance. There is also a [docker image](https://github.com/ddmler/docker-clingo/) available to run this on.
