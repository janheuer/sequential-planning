direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.
nextto((X,Y),(X',Y'),(X+X',Y+Y')) :- position((X,Y)), direction((X',Y')), position((X+X',Y+Y')).

#program step(t).

 {    move(R,D,t) : direction(D) ;
    pickup(R,S,t) : isShelf(S)   ;
   putdown(R,S,t) : isShelf(S)   } 1 :- isRobot(R).

waits(R,t) :- not pickup(R,_,t), not putdown(R,_,t), not move(R,_,t), isRobot(R).

% - move/3 ----------------------------------------------------------------------
position(R,C,t) :- move(R,D,t),        position(R,C',t-1),     nextto(C',D,C).
                :- move(R,D,t),        position(R,C ,t-1), not nextto(C ,D,_).

% - pickup/3 --------------------------------------------------------------------
 carries(R,S,t) :- pickup(R,S,t).
                :- pickup(R,S,t),       carries(R,_,t-1).
                :- pickup(R,S,t),       carries(_,S,t-1).
                :- pickup(R,S,t),      position(R,C,t-1), not position(S,C,t-1).
                :- pickup(R,S,t),      position(S,C,t-1), not position(R,C,t-1).

% - putdown/3 -------------------------------------------------------------------
                :- putdown(R,S,t), not carries(R,S ,t-1).

% - serves/4 --------------------------------------------------------------------
serves(R,S,P,t) :- position(R,C,t), carries(R,S,t), position(P,C), strategy.

% - inertia ---------------------------------------------------------------------
position(R,C,t) :- position(R,C,t-1), not    move(R,_,t), isRobot(R).
 carries(R,S,t) :-  carries(R,S,t-1), not putdown(R,S,t).

% - (in)direct effects ----------------------------------------------------------
position(S,C,t) :- position(R,C,t  ),     carries(R,S,t).
position(S,C,t) :- position(S,C,t-1), not carries(_,S,t), isShelf(S).

% - edge collision --------------------------------------------------------------
moveto(C',C,t) :- nextto(C',D,C), position(R,C',t-1), move(R,D,t).
 :- moveto(C',C,t), moveto(C,C',t), C < C'.

% - vertex collision ------------------------------------------------------------
 :- { position(R,C,t) : isRobot(R) }  > 1, position(C).
 :- { position(S,C,t) : isShelf(S) }  > 1, position(C).

% - auxiliaries -----------------------------------------------------------------
 :- { position(R,C,t) } != 1, isRobot(R).
 :- { position(S,C,t) } != 1, isShelf(S).

 :- { carries(R,S,t) } > 1, isRobot(R).
 :- { carries(R,S,t) } > 1, isShelf(S).
