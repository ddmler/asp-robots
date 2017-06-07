# Distributed Encoding

#### Run

First edit master-python.lp and change the full path to the clingo executable.

```sh
clingo master-python.lp master.lp instance.lp
```

The master distributes orders to workers and starts a worker for each robot in the instance. They plan the fulfillment of their orders, after that the checker looks for collisions and other problems and the workers have to replan to fix those problems.
