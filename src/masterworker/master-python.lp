#script (python)

import clingo
import re
import os

def main(prg):

    # full path to clingo executable
    clingopath = "/home/fabian/clingo/build/release/clingo"

    def start_worker(i, b):
        print("worker " + str(i))
        # write workerid into a file, so that the robot knows his ID
        os.system("echo \"self(" + str(i) + ").\" > workerid.lp")
        if b==False:
            # without checkerplan
            os.system(clingopath + " 1 masterplan.lp worker.lp workerid.lp worker-python.lp")
        else:
            # with checkerplan
            os.system(clingopath + " 1 checkerplan.lp masterplan.lp worker.lp workerid.lp worker-python.lp")
        os.system("rm workerid.lp")
        print("------------------------------------------")

    def on_model(m):
        global masterplan, maxrob
        # on model from master get maximum robot id and the masterplan
        masterplan = ". ".join(map(str, m.symbols(shown=True))) + "."
        regex = r"roboto\((\d+)\)"
        list = re.findall(regex, masterplan)
        maxrob = max(list)

    # start master
    prg.ground([("base", [])])
    print("------------------------------------------")
    print("master")
    prg.solve(on_model=on_model)
    print("------------------------------------------")

    # write model from master to masterplan
    os.system("echo \"" + masterplan + "\" > masterplan.lp")

    # start one worker per robot
    for i in range(1,int(maxrob)+1):
        start_worker(i,False)

    # start checker on combined workerplans
    os.system(clingopath + " 1 checker.lp  checker-python.lp workerplan.lp ")

    # start robots again as long as there are bugs from the checker
    while open('checkerplan.lp').readline().strip() != "": # ändern auf inhalt der datei == ""
        os.system("rm workerplan.lp")
        for i in range(1,int(maxrob)+1):
            start_worker(i,True)
        os.system(clingopath + " 1 checker.lp  checker-python.lp workerplan.lp ")



    # remove stuff and rename the combined plans to solution.lp
    os.system("mv workerplan.lp solution.lp && rm masterplan.lp && rm checkerplan.lp")

#end.
