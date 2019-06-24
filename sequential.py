import clingo
import argparse
import re
from time import time

class SequentialSolver(object):
    def __init__(self):
        """Initialization, call planning function, and output"""
        # arguments
        parser = argparse.ArgumentParser(description = "Sequential planning in the asprilo framework", usage='%(prog)s [options] instance')
        parser.add_argument("-d", "--debug", default=False, action="store_true", help="Additional debugging outputs")
        parser.add_argument("-p", "--parallel", default=False, action="store_true", help="Plan parallel (for comparison to sequential mode)")
        parser.add_argument("-f", "--file-output", default=False, action="store_true", help="Output model to file (if the instance is 'instance.lp' the output file will be 'instance_plan.lp' or 'instance_plan_p.lp' for the parallel mode)")
        parser.add_argument("-b", "--benchmark", default=False, action="store_true", help="Benchmarking mode, time will be taken and appended to file time.txt, plan length will be appended to length.txt")
        parser.add_argument("instance", type=str, help="Instance to be solved (sequential planning additionally needs an assignment of order to robots)")
        parser.add_argument("-o", "--order-assignment", default="", type=str, help="File containing the order assignments, if no file is given and the instance is 'instance.lp', 'instanceo.lp' will be used")
        args = parser.parse_args()

        # initializing variables
        self.debug = args.debug
        self.parallel = args.parallel
        self.file_output = args.file_output
        self.benchmark = args.benchmark
        self.instance = args.instance
        self.assignment_file = args.order_assignment if args.order_assignment != "" else self.instance[:-3]+"o.lp"
        self.encoding = "./sequential_encodings/encoding.lp" if not args.parallel else "./parallel_encodings/encoding.lp"

        # saves the overall model
        self.model = []
        # save individual robot plans
        self.plans = {}
        # maximum plan length of all robots
        self.plan_length = 0
        # which robot is the current one to plan
        self.current_robot = 1

        # call run method which does the planning
        # if benchmark option is specified this is timed
        if self.benchmark:
            ts = time()
        if not self.parallel:
            self.run()
        else:
            self.run_parallel()
        if self.benchmark:
            tf = time()
            # output time
            f_time = open("./time.txt", "a")
            f_time.write(str(tf-ts)+"\n")
            f_time.close()
            # output plan length
            f_length = open("./length.txt", "a")
            f_length.write(str(self.plan_length)+"\n")
            f_length.close()

        # output plan
        if not self.file_output:
            self.output_model()
        else:
            self.output_model_file()

    def get_assignment(self):
        """Parse and save assignment of orders from self.assignment_file"""
        # order assignments are save in a dict
        # keys: robot id
        # value: [order id]
        self.assignment = {}

        with open(self.assignment_file) as f:
            # remove all whitespace and '.'
            atoms = [re.sub(r'\s+', '', line).strip(".") for line in f]

        for atom in atoms:
            # atom has form: assignOrder(robot(N),order(O))
            # find all integers
            args = re.findall(r'\d+', atom)
            # args[0] is robot number, args[1] is order id
            # create new key for the robot if necessary
            if int(args[0]) not in self.assignment:
                    self.assignment[int(args[0])] = []
            # add order to list of orders for the robot
            self.assignment[int(args[0])].append(int(args[1]))

        # get number of robots
        self.number_robots = len(self.assignment)

    def get(self, val, default):
        """Helper function for incremental solving"""
        return val if val != None else default

    def get_plan(self, model):
        """Save plan in overall plan as well as individual plan of the current robot"""
        # self.plans saves part of the plan that is used as input for the next robots
        self.plans[self.current_robot] = []
        for atom in model.symbols(shown=True):
            if ((atom.name == "position") or
                (atom.name == "move")     or
                (atom.name == "pickup")   or
                (atom.name == "putdown")  or
                (atom.name == "carries")):
                self.plans[self.current_robot].append(atom)
            else:
                # all other atoms are saved in the overall plan, but inits only once
                if (atom.name == "init" and self.current_robot != 1):
                    continue
                self.model.append(atom)

    def plan(self):
        """Planning function for the sequential mode, based on clingo incremental mode
        Additionally adds atoms for which robot and for which orders a plan needs to be found,
        and plans of all previous robots are added as input"""
        prg = clingo.Control(arguments=["-Wnone"])
        prg.load(self.encoding)
        prg.load(self.instance)
        # which robot is planning
        prg.add("base", [], "planning(robot("+str(self.current_robot)+")).")
        # which orders need to be planned
        for oid in self.assignment[self.current_robot]:
            prg.add("base", [], "process(order("+str(oid)+")).")
        prg.add("base", [], "planLength("+str(self.plan_length)+").")

        # add plans of all previous robots
        if self.current_robot != 1:
            for i in range(1,self.current_robot):
                for atom in self.plans[i]:
                    prg.add("base", [], str(atom)+".")

        # incremental solving
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

        # save maximum plan length
        if self.plan_length < step - 1:
            self.plan_length = step - 1

    def get_parallel_plan(self, model):
        """Save plan in overall plan"""
        for atom in model.symbols(shown=True):
            self.model.append(atom)

    def parallel_plan(self):
        """Planning function for the parallel mode, based on clingo incremental mode
        Here no additional inputs are needed"""
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

        # save plan length for benchmark output
        self.plan_length = step - 1

    def output_model(self):
        """Print out overall model"""
        for atom in self.model:
            print(str(atom)+".")

    def output_model_file(self):
        """Output overall plan to file"""
        if self.parallel:
            name = self.instance[:-3]+"_plan_p.lp"
        else:
            name = self.instance[:-3]+"_plan.lp"
        f = open(name,"w")
        for atom in self.model:
            f.write(str(atom)+".\n")
        f.close()

    def run(self):
        """Sequential planning function: get assignments and then plan sequentially"""
        self.get_assignment()
        for i in range(1,self.number_robots+1):
            if self.debug:
                print(self.plan_length)
            self.plan()
            if self.debug:
                print(self.plans[i])
            self.current_robot += 1

    def run_parallel(self):
        """Parallel planning function"""
        self.parallel_plan()

if __name__ == "__main__":
    SequentialSolver()
