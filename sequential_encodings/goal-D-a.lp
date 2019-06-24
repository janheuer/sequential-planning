#include "./quantities.lp".

#program check(t).
#external query(t).

:- ordered(_,_,1,t), query(t).

:- carries(_,_,t), query(t).

:- position(_,C), position(_,C,t), query(t).

% additional constraints for cases where the plan length of the planning robot
% is less than the plan length of the input plan(s) of the other robot(s)
% so constraints after the plan of the planning robot is already finished

% planning robot cant be on positon that a non planning robot moves onto
:- position(R1,C,t), planning(R1),
   position(R2,C,T), not planning(R2),
   T=t..H, planLength(H), query(t).

% shelves that were used by the planning robot cant be on position that a non planning moves onto while carrying a shelf
:- position(S,C,t), carries(R1,S,_), planning(R1),
   position(R2,C,T), carries(R2,_,T), not planning(R2),
   T=t..H, planLength(H), query(t).
:- position(S,C,t), carries(R1,S,_), planning(R1),
   position(R2,C,T), putdown(R2,_,T), not planning(R2),
   T=t..H, planLength(H), query(t).

% planning robots cant finish in highway nodes
:- position(R,C,t), planning(R), highway(C), query(t).
