#script (python)

import clingo
import re
import os

def main(prg):

    clingopath = "/home/fabian/clingo/build/release/clingo"

    def start_worker(i, b):
        print("worker " + str(i))
        os.system("echo \"self(" + str(i) + ").\" > workerid.lp")
        if b==False:
            os.system(clingopath + " 1 masterplan.lp worker.lp workerid.lp worker-python.lp")
        else:
            os.system(clingopath + " 1 checkerplan.lp masterplan.lp worker.lp workerid.lp worker-python.lp")
        os.system("rm workerid.lp")
        print("------------------------------------------")

    def on_model(m):
        global masterplan, maxrob
        masterplan = ". ".join(map(str, m.symbols(shown=True))) + "."
        regex = r"roboto\((\d+)\)"
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
        start_worker(i,False)

    os.system(clingopath + " 1 checker.lp  checker-python.lp workerplan.lp ")

    while open('checkerplan.lp').readline().strip() != "": # ändern auf inhalt der datei == ""
        os.system("rm workerplan.lp")
        for i in range(1,int(maxrob)+1):
            start_worker(i,True)
        os.system(clingopath + " 1 checker.lp  checker-python.lp workerplan.lp ")




    os.system("mv workerplan.lp solution.lp && rm masterplan.lp && rm checkerplan.lp")

#end.