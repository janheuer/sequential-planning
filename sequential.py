import clingo
import argparse
import re
from time import time

class SequentialSolver(object):
    def __init__(self):
        # arguments
        parser = argparse.ArgumentParser(description = "Sequential planning in the asprilo framework", usage='%(prog)s [options] instance')
        parser.add_argument("-d", "--debug", default=False, action="store_true", help="Additional debugging outputs")
        parser.add_argument("-p", "--parallel", default=False, action="store_true", help="Plan parallel (for comparison to sequential mode)")
        parser.add_argument("-f", "--file-output", default=False, action="store_true", help="Output model to file (if the instance is 'instance.lp' the output file will be 'instance_plan.lp' or 'instance_plan_p.lp' for the parallel mode)")
        parser.add_argument("-b", "--benchmark", default=False, action="store_true", help="Benchmarking mode, time will be taken and appended to file time.txt, plan length will be appended to length.txt")
        parser.add_argument("instance", type=str, help="Instance to be solved")
        args = parser.parse_args()

        # initializing variables
        self.debug = args.debug
        self.parallel = args.parallel
        self.file_output = args.file_output
        self.benchmark = args.benchmark
        self.instance = args.instance
        self.assignment_file = args.instance[:-3]+"o.lp"
        self.encoding = "./sequential_encodings/encoding.lp" if not args.parallel else "./parallel_encodings/encoding.lp"

        # saves the overall model
        self.model = []
        # save individual robot plans
        self.plans = {}
        # maximum plan length of all robots
        self.plan_length = 0
        # which robot is the current one to plan
        self.current_robot = 1

        if self.benchmark:
            ts = time()
        if not self.parallel:
            self.run()
        else:
            self.run_parallel()
        if self.benchmark:
            tf = time()
            f_time = open("./time.txt", "a")
            f_time.write(str(tf-ts)+"\n")
            f_time.close()
            f_length = open("./length.txt", "a")
            f_length.write(str(self.plan_length)+"\n")
            f_length.close()

        if not self.file_output:
            self.output_model()
        else:
            self.output_model_file()

    def get_assignment(self):
        self.assignment = {}

        with open(self.assignment_file) as f:
            atoms = [re.sub(r'\s+', '', line).strip(".") for line in f]

        for atom in atoms:
            args = re.findall(r'\d+', atom)
            if int(args[0]) not in self.assignment:
                    self.assignment[int(args[0])] = []
            self.assignment[int(args[0])].append(int(args[1]))

        self.number_robots = len(self.assignment)

    def get(self, val, default):
        return val if val != None else default

    def get_plan(self, model):
        self.plans[self.current_robot] = []
        for atom in model.symbols(shown=True):
            if ((atom.name == "position") or
                (atom.name == "move")     or
                (atom.name == "pickup")   or
                (atom.name == "putdown")  or
                (atom.name == "carries")):
                self.plans[self.current_robot].append(atom)
            else:
                if (atom.name == "init" and self.current_robot != 1):
                    continue
                self.model.append(atom)

    def plan(self):
        prg = clingo.Control(arguments=["-Wnone"])
        prg.load(self.encoding)
        prg.load(self.instance)
        prg.add("base", [], "planning(robot("+str(self.current_robot)+")).")
        for oid in self.assignment[self.current_robot]:
            prg.add("base", [], "process(order("+str(oid)+")).")
        prg.add("base", [], "planLength("+str(self.plan_length)+").")

        if self.current_robot != 1:
            for i in range(1,self.current_robot):
                for atom in self.plans[i]:
                    prg.add("base", [], str(atom)+".")

        imin   = self.get(prg.get_const("imin"), clingo.Number(0))
        imax   = prg.get_const("imax")
        istop  = self.get(prg.get_const("istop"), clingo.String("SAT"))

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
            ret,step = prg.solve(on_model=self.get_plan), step+1

            if self.debug:
                print(str(step-1))

        if self.plan_length < step - 1:
            self.plan_length = step - 1

    def parallel_plan(self):
        prg = clingo.Control(arguments=["-Wnone"])
        prg.load(self.encoding)
        prg.load(self.instance)

        imin   = self.get(prg.get_const("imin"), clingo.Number(0))
        imax   = prg.get_const("imax")
        istop  = self.get(prg.get_const("istop"), clingo.String("SAT"))

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
            ret,step = prg.solve(on_model=self.get_parallel_plan), step+1

        self.plan_length = step - 1

    def get_parallel_plan(self, model):
        for atom in model.symbols(shown=True):
            self.model.append(atom)

    def output_model(self):
        for atom in self.model:
            print(str(atom)+".")

    def output_model_file(self):
        if self.parallel:
            name = self.instance[:-3]+"_plan_p.lp"
        else:
            name = self.instance[:-3]+"_plan.lp"
        f = open(name,"w")
        for atom in self.model:
            f.write(str(atom)+".\n")
        f.close()

    def run(self):
        self.get_assignment()
        for i in range(1,self.number_robots+1):
            if self.debug:
                print(self.plan_length)
            self.plan()
            if self.debug:
                print(self.plans[i])
            self.current_robot += 1

    def run_parallel(self):
        self.parallel_plan()

if __name__ == "__main__":
    SequentialSolver()
