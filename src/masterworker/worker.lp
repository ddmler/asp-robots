time(1..10).
{ robmove(ID,-1..1, -1..1, T) : time(T) } :- self(ID).
robmove(ID, 0, 1, T) :- self(ID), bug(ID, T).
robpos(ID,X,Y,1):-rob(ID,X,Y), self(ID).
robpos(ID,A+X,B+Y,T+1):-robmove(ID, A, B, T), robpos(ID,X,Y,T).
maxt(T):- T=#max{Z: robpos(_,_,_,Z)}.
:- maxt(T), not robpos(ID,N,M,T), goal(ID,N,M), self(ID).
:- robmove(ID,_, _, T), not robmove(ID,_, _, T-1), T!=1.
:- robmove(ID,X, Y, T), robmove(ID,A, B, T), X!=A.
:- robmove(ID,X, Y, T), robmove(ID,A, B, T), Y!=B.

#show robmove/4.
#show robpos/4.