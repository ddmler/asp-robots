#script (python)

import clingo
import os

def main(prg):
    def on_model(m):
        # extract output from checker
        checkerplan = ". ".join(map(str, m.symbols(shown=True))) + "."
        if checkerplan != ".":
            # add bugs from checker to checkerplan and masterplan (to persist them for later rounds)
        	os.system("echo \"" + checkerplan + "\" > checkerplan.lp && echo \"" + checkerplan + "\" >> masterplan.lp")
        else:
        	os.system("echo \"\" > checkerplan.lp")

    # execute checker
    prg.ground([("base", [])])
    print("------------------------------------------")
    print("checker")
    prg.solve(on_model=on_model)
    print("------------------------------------------")

#end.
