#program base.

% Get the number of robots and orders of the instance
maxrob(X) :- X=#max{T: init(object(robot,T),value(at,(_,_))) }.
maxorder(Y) :- Y=#max{A:  init(object(order,A),value(pickingStation,_)) }.

% Define roboto for master-python.lp script
roboto(1..X) :- maxrob(X).

% assign orders to robotos fairly, so that no robot has more than 1 order more assigned than the other ones
{roborder(RID,OID):roboto(RID),init(object(order,OID),value(pickingStation,_)) }=A :- maxorder(A).
:- roboto(R),A=#count{OID: roborder(R,OID)}, B=#count{Z:roborder(T,Z)}, roboto(T), |A-B|>1.

% all orders need to be assigned
:- init(object(order,OID),value(pickingStation,_)), not roborder(_,OID).


