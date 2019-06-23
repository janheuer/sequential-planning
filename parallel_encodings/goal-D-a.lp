#include "./quantities.lp".

#program check(t).
#external query(t).

:- ordered(_,_,1,t), query(t).

:- carries(_,_,t), query(t).

:- position(_,C), position(_,C,t), query(t).
