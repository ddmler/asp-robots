#program base.


maxrob(X) :- X=#max{T: init(object(robot,T),value(at,(_,_))) }.
maxorder(Y) :- Y=#max{A:  init(object(order,A),value(pickingStation,_)) }.

roboto(1..X) :- maxrob(X).

%assign orders
%{roborder(RID, OID) : init(object(order,OID),value(pickingStation,_)), RID = X/OID, maxrob(X)}=A :- X>=A, maxorder(A), maxrob(X).
%{roborder(RID, OID) : init(object(order,OID),value(pickingStation,_)), RID = 1..X, maxrob(X)}=A :- X<A, maxorder(A), maxrob(X).

{roborder(RID,OID):roboto(RID),init(object(order,OID),value(pickingStation,_)) }=A :- maxorder(A).
:- roboto(R),A=#count{OID: roborder(R,OID)}, B=#count{Z:roborder(T,Z)}, roboto(T), |A-B|>1.

%all orders need to be assigned
:- init(object(order,OID),value(pickingStation,_)), not roborder(_,OID).

