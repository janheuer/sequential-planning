% only for orders that need to be processed as indicated by process(order(O))
ordered(order(O),product(A),N) :- init(object(order,O),  value(line,(A,N))), process(order(O)).
shelved(shelf(S),product(A),N) :- init(object(product,A),value(on,  (S,N))), ordered(order(O),product(A)).

limit(A,O,S,(N+N'-|N-N'|)/2) :- ordered(O,A,N), shelved(S,A,N').

ordered(O,A,1..N,0) :- ordered(O,A,N).
shelved(S,A,1..N,0) :- shelved(S,A,N).

#program step(t).

 { process(A,O,S,1..M,t) : limit(A,O,S,M) } 1.

process(A,O,M,t) :- process(A,O,_,M,t).
process(A,S,M,t) :- process(A,_,S,M,t).

process(A,shelf(S),t) :- process(A,shelf(S),M,t).
process(S,t)          :- process(A,S,t).

:- process(A,order(O),M,t), not ordered(order(O),A,M,t-1).
:- process(A,shelf(S),M,t), not shelved(shelf(S),A,M,t-1).

ordered(O,A,N-M,t) :- ordered(O,A,N,t-1), process(A,O,M,t), M < N.
shelved(S,A,N-M,t) :- shelved(S,A,N,t-1), process(A,S,M,t), M < N.

ordered(O,A,N,  t) :- ordered(O,A,N,t-1), not process(A,O,_,t).
shelved(S,A,N,  t) :- shelved(S,A,N,t-1), not process(A,S,t).

:- process(_,O,S,_,t), target(O,P), position(P,C), not position(S,C,t-1).


:- process(S,t), not carries(_,S,t-1).
:- process(S,t), carries(R,S,t-1), not waits(R,t).
