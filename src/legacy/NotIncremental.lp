% init
init(object(node,1),value(at,(1,1))).
init(object(node,2),value(at,(2,1))).
init(object(node,3),value(at,(3,1))).
init(object(node,4),value(at,(1,2))).
init(object(node,5),value(at,(2,2))).
init(object(node,6),value(at,(3,2))).
init(object(node,7),value(at,(1,3))).
init(object(node,8),value(at,(2,3))).
init(object(node,9),value(at,(3,3))).
init(object(order,1),value(line,(1,1))).
init(object(order,1),value(pickingStation,1)).
init(object(order,2),value(line,(2,1))).
init(object(order,2),value(pickingStation,1)).
init(object(pickingStation,1),value(at,(1,2))).
init(object(product,1),value(on,(1,1))).
init(object(product,2),value(on,(1,1))).
init(object(robot,1),value(at,(3,1))).
init(object(shelf,1),value(at,(2,2))).
init(object(shelf,2),value(at,(3,2))).

%%%%%%% Constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const imin   = 0.
#const imax   = 13	. 

%%%%%%% base program %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

time(0).
turn(1..50).
maxT(T) :- T=#max{Z: time(Z)}.
% ACTIONS
action(move(0,1)).
action(move(1,0)).
action(move(0,-1)).
action(move(-1,0)).
action(pickup).
action(putdown).
action(deliver(RQID, PRID, AMT)) :- init(request(RQID, PRID, StationID, RQAMT)), AMT = 1..RQAMT.
% INITIAL STATE
holds(F,0) :- init(F).

% casting
node(X,Y) :- init(object(node, ID), value(at, (X, Y))). 
highway(X,Y) :- init(object(highway, ID), value(at, (X, Y))). 
init(at(robot(ID), node(X, Y))) :- init(object(robot, ID), value(at, (X, Y))).
init(at(shelf(ID), node(X, Y))) :- init(object(shelf, ID), value(at, (X, Y))).
init(at(pickingstation(ID), node(X, Y))) :- init(object(pickingStation, ID), value(at, (X, Y))).
init(on(ID, ShelfID, Amount)) :- init(object(product, ID), value(on, (ShelfID, Amount))).
init(request(ID, Item, StationID, Amount)) :- init(object(order, ID), value(line, (Item, Amount))), init(object(order,ID),value(pickingStation, StationID)).
init(carries(RID,SID)) :- init(object(robot, RID), value(carries, SID)).
robot(Robot) :- init(at(robot(Robot), node(_,_))).
shelf(Shelf) :- init(at(shelf(Shelf), node(_,_))).
pickingstation(Pickingstation) :- init(at(pickingstation(Pickingstation), node(_,_))).
product(Product) :- init(on(Product, _, _)).

% max. X-Coordinate of field
maxX(X) :- X=#max{S:init(object(node,_),value(at,(S,Y)))}.

% max. Y-Coordinate of field
maxY(Y) :- Y=#max{S:init(object(node,_),value(at,(X,S)))}.

%%%%%%% action handling %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% cast internal plan format to output plan format
occurs(object(robot, RID), action(move, (DX,DY)), T) :- occurs(robot(RID),move(DX,DY),T).
occurs(object(robot, RID), action(pickup, ()), T) :- occurs(robot(RID),pickup,T).
occurs(object(robot, RID), action(putdown, ()), T) :- occurs(robot(RID),putdown,T).
occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), T) :- occurs(robot(RID),deliver(RQID, PRID, RQAMT),T).



{ occurs(robot(RID),A,T+1):action(A) } 1 :- robot(RID),time(T), T<imax

% Generating actions with preconditions 
{ occurs(robot(RID),A,T+1):action(A) } 1 :- robot(RID),time(T), T<imax, holds(request(_,_,_,_),T). 
%occurs(robot(1),move(0,-1),2).
%occurs(robot(1),pickup,1).
%occurs(robot(1),move(1,0),3).
time(T) :- 1{occurs(_,_,T)}, time(T-1).
fin(RID,T) :- occurs(robot(RID),A,T), not occurs(robot(RID),B,T+1), action(B).

mov(X+DX,Y+DY):- holds(at(robot(RID),node(X,Y)),T), occurs(robot(RID),move(DX,DY),T+1).
:- mov(X,Y), not node(X,Y).
:- holds(at(robot(RID),node(X,Y)),T-1), not holds(at(shelf(_),node(X,Y)),T-1), occurs(robot(RID),pickup,T).





:- occurs(robot(RID),pickup,T), not pick(robot(RID),pickup,T).
:- occurs(robot(RID),putdown,T), not put(robot(RID),putdown,T).
:- occurs(robot(RID),deliver(RQID, PRID, RQAMT),T), not del(robot(RID),deliver(RQID, PRID, RQAMT),T).
%%	ACTION - PRECONDITIONS
%       The preconditions for move(DX,DY), pickup and putdown as described at the top under %%%ACTIONS%%%.

pick(robot(RID),pickup,T)       	:-	holds(at(robot(RID),node(X,Y)),T-1),
            							holds(at(shelf(SID),node(X,Y)),T-1),
            							not holds(carries(RID,_),T-1).
put(robot(RID),putdown,T)	        :-	holds(carries(RID,_),T-1).

del(robot(RID), deliver(RQID, PRID, RQAMT),T) :- 
                                        RQAMT=1..AMT,
                                        holds(request(RQID,PRID,PSID,_),T-1),
                                        holds(carries(RID,_),T-1),
										holds(on(PRID,SID,AMT),T-1),
										holds(at(shelf(SID),node(X,Y)),T-1),
										holds(at(pickingstation(PSID),node(X,Y)),T-1),
                                        holds(at(robot(RID), node(X,Y)), T-1).


holds(at(robot(RID),node(X+DX,Y+DY)),T)	:-	node(X+DX,Y+DY),
										holds(at(robot(RID),node(X,Y)),T-1),
										occurs(robot(RID),move(DX,DY),T).
holds(at(shelf(SID),node(X+DX,Y+DY)),T)	:-	node(X+DX,Y+DY),
										holds(at(robot(RID),node(X,Y)),T-1),
										occurs(robot(RID),move(DX,DY),T),
										holds(carries(RID,SID),T-1).
holds(carries(RID,SID),T)			:-	holds(at(robot(RID),node(X,Y)),T),
										holds(at(shelf(SID),node(X,Y)),T),
										occurs(robot(RID),pickup,T).
moved(robot(RID),T)                 :-  occurs(robot(RID),move(_,_),T), robot(RID).


holds(on(PRID,SID,AMT-RQAMT),T)		:-	AMT>RQAMT,
										holds(on(PRID,SID,AMT),T-1),
                                        holds(carries(RID,SID),T-1),
                                        occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), T).

holds(request(RQID,PRID,PSID,RQAMT-AMT),T):-	AMT<RQAMT,
                                        holds(request(RQID,PRID,PSID,RQAMT),T-1),
                                        occurs(object(robot, RID), action(deliver, (RQID, PRID, AMT)), T).

holds(done(RQID,PRID,PSID,RQAMT),T)	:-	holds(request(RQID,PRID,PSID,RQAMT),T-1),
                                        occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), T).

%%	INERTIA
holds(F,T) :- holds(F,T-1), not ab(F,T), T<imax, time(T).

%%	INERTIA - ABNORMALS
%       Anything that changes is abnormal and won'T be handled by inertia.
ab(at(robot(RID),node(X,Y)),T) 	:-	holds(at(robot(RID),node(X,Y)),T-1),
								moved(robot(RID),T).								
ab(at(shelf(SID),node(X,Y)),T) 	:-	holds(at(shelf(SID),node(X,Y)),T-1),
								holds(carries(RID,SID),T-1),
								moved(robot(RID),T).
ab(carries(RID,SID),T)		:-	holds(carries(RID,SID),T-1),
								occurs(robot(RID),putdown,T).
ab(on(PRID,SID,AMT),T)		:-	holds(on(PRID,SID,AMT),T-1),
                                holds(carries(RID,SID),T-1),
                                occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), T).
ab(request(RQID,PRID,PID,RQAMT),T) :-	holds(request(RQID,PRID,PID,RQAMT),T-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, AMT)), T).
ab(request(RQID,PRID,PID,RQAMT),T) :-	holds(done(RQID,PRID,PID,RQAMT),T).

goal(T):- time(T), not holds(request(_,_,_,_),T).

%:- maxT(T), not goal(T).
:- init(request(RQID, PRID, PSID, Amount)), not holds(done(RQID,PRID,PSID,_),_).
%%%%%%% output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


:-	holds(at(robot(I),node(X,Y)),T), holds(at(robot(J),node(X,Y)),T), I!=J.
:-	holds(at(shelf(I),node(X,Y)),T), holds(at(shelf(J),node(X,Y)),T), I!=J.

:-	holds(at(robot(I),node(X,Y)),T), holds(at(robot(I),node(A,B)),T), A!=X,B!=Y.
:-	holds(at(shelf(I),node(X,Y)),T), holds(at(shelf(I),node(A,B)),T), A!=X,B!=Y.

%       Two robots can'T swap places as they would crash.
:-	holds(at(robot(RID1),node(X1,Y1)),T), holds(at(robot(RID2),node(X2,Y2)),T),
	holds(at(robot(RID1),node(X2,Y2)),T-1), holds(at(robot(RID2),node(X1,Y1)),T-1),
	RID1<RID2, |X2-X1|+|Y2-Y1|==1.

%       On higway nodes shelfs can'T be putdown and robots can'T do nothing for more than one time step.
:-  highway(X,Y), not node(X,Y).
:-  highway(X,Y), holds(at(robot(RID),node(X,Y)),T), occurs(robot(RID),putdown,T).
:-  highway(X,Y), holds(at(robot(RID),node(X,Y)),T), not moved(robot(RID),T-1), not moved(robot(RID),T), not init(at(robot(RID),node(X,Y))).

%       Small scope loop preventions preventing opposite actions at adjacent time points.
:- occurs(robot(RID),move(DX,DY),T), occurs(robot(RID),move(-DX,-DY),T-1).
:- occurs(robot(RID),pickup,T), occurs(robot(RID),putdown,T-1).
:- occurs(robot(RID),putdown,T), occurs(robot(RID),pickup,T-1).



%:- not holds(at(robot(2),node(1,1)),T), maxT(T).
%:- not holds(at(robot(1),node(2,1)),T), maxT(T).
%:- not holds(carries(1,2),T), maxT(T).



%#minimize{ 1, RID, T : occurs(robot(RID),_,T)}.

#show occurs/3.







