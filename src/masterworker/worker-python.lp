#script (python)

import clingo
import os

def get(val, default):
    return val if val != None else default

def main(prg):
    def on_model(m):
        workerplan = ". ".join(map(str, m.symbols(shown=True))) + "."
        os.system("echo \"" + workerplan + "\" >> workerplan.lp")

    imin   = get(prg.get_const("imin"), clingo.Number(0))
    imax   = prg.get_const("imax")
    istop  = get(prg.get_const("istop"), clingo.String("SAT"))

    step, ret = 0, None
    while ((imax is None or step < imax.number) and
           (step == 0 or step < imin.number or (
              (istop.string == "SAT"     and not ret.satisfiable) or
              (istop.string == "UNSAT"   and not ret.unsatisfiable) or
              (istop.string == "UNKNOWN" and not ret.unknown)))):
        parts = []
        parts.append(("check", [step]))
        if step > 0:
            prg.release_external(clingo.Function("query", [step-1]))
            parts.append(("step", [step]))
            prg.cleanup()
        else:
            parts.append(("base", []))
        prg.ground(parts)
        prg.assign_external(clingo.Function("query", [step]), True)
        ret, step = prg.solve(on_model=on_model), step+1
#end.

#program check(t).
#external query(t).
