#script (python)

import clingo
import os

def main(prg):
    def on_model(m):
        checkerplan = ". ".join(map(str, m.symbols(shown=True))) + "."
        os.system("echo \"" + checkerplan + "\" > checkerplan.lp")

    prg.ground([("base", [])])
    print("------------------------------------------")
    print("checker")
    prg.solve(on_model=on_model)
    print("------------------------------------------")

#end.
