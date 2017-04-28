#script (python)

import clingo
import re
import os

def main(prg):
    def start_worker(i):
        print("worker " + str(i))
        os.system("echo \"self(" + str(i) + ").\" > workerid.lp")
        os.system("/home/enrico/Schreibtisch/clingo/build/release/clingo 1 masterplan.lp worker.lp workerid.lp worker-python.lp")
        os.system("rm workerid.lp")
        print("------------------------------------------")

    def on_model(m):
        global masterplan, maxrob
        masterplan = ". ".join(map(str, m.symbols(shown=True))) + "."
        regex = r"rob\((\d+)\)"
        list = re.findall(regex, masterplan)
        maxrob = max(list)

    prg.ground([("base", [])])
    print("------------------------------------------")
    print("master")
    prg.solve(on_model=on_model)
    print("------------------------------------------")

    # worker
    os.system("echo \"" + masterplan + "\" > masterplan.lp")

    for i in range(1,int(maxrob)+1):
        start_worker(i)

    os.system("/home/enrico/Schreibtisch/clingo/build/release/clingo 1 checker.lp  checker-python.lp workerplan.lp ")





    os.system("rm masterplan.lp && rm workerplan.lp && rm checkerplan.lp")

#end.
