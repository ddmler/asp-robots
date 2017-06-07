%%%%%%% idea %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% the basic idea is the modelling of an automaton like in theoretical computer science.
% attributes like location of a robot and shelf represent a state at time t of the automaton as a set
% depending on the state there are varying actions possible for each robot
% actions may be seen as transitions between states
% thus actions may change the state of an automaton (e.g. change of the location of a robot)
% reaching the state of fulfillment of all orders is the goal of this encoding
%%%%%%% references %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% parts of this code were used from the original encoding of Obermeier
%%%%%%% advices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Command line option: use --configuration=crafty for better performance
% Command line option: use 1 (e.g. clingo 1 encoding.lp) for faster solving
% Command line option: use --heuristic=Vsids or --heuristic=Domain for better performance
% Comment minimize for faster solving
% Solving via command line needs the incmode-python.lp script (e.g. clingo encoding.lp python.lp)
%%%%%%% constants %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#const incremental = true.
#const istop  = "SAT".
#const imin   = 0.
#const imax   = 30.

%%%%%%% base program (borrowed from original and edited) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#program base.

%% initial state
holds(F,0) :- init(F).

%%cast external instance format to internal instance format
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

% relaxation 2 casting
init(on(ID, ShelfID)) :- init(on(ID, ShelfID, Amount)).

%%%%%%% steps %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#program step(t).

%%cast internal plan format to output plan format (borrowed from original)
occurs(object(robot, RID), action(move, (DX,DY)), t) :- occurs(robot(RID),move(DX,DY),t).
occurs(object(robot, RID), action(pickup, ()), t) :- occurs(robot(RID),pickup,t).
occurs(object(robot, RID), action(putdown, ()), t) :- occurs(robot(RID),putdown,t).
occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t) :- occurs(robot(RID),deliver(RQID, PRID, RQAMT),t).

% define all possible acions for a robot at step t for each case

possmove(-1,0,RID,t) :- holds(at(robot(RID),node(X,Y)),t), node(X-1,Y).
possmove(1,0,RID,t) :- holds(at(robot(RID),node(X,Y)),t), node(X+1,Y).
possmove(0,1,RID,t) :- holds(at(robot(RID),node(X,Y)),t), node(X,Y+1).
possmove(0,-1,RID,t) :- holds(at(robot(RID),node(X,Y)),t), node(X,Y-1).

posscasea(RID,move(X,Y),t) :- possmove(X,Y,RID,t-1).

posscaseb(RID,move(X,Y),t) :- possmove(X,Y,RID,t-1).
posscaseb(RID,pickup,t) :- robot(RID).

posscasec(RID,move(X,Y),t) :- possmove(X,Y,RID,t-1).
posscasec(RID,putdown,t) :- robot(RID).

posscased(RID,move(X,Y),t) :- possmove(X,Y,RID,t-1).
posscased(RID,deliver(RQID, PRID, RQAMT),t) :- robot(RID),init(request(RQID, PRID, StationID, RQAMT)).
% defining cases for different actions

casea(t,RID) :- holds(at(robot(RID),node(X,Y)),t), not holds(at(shelf(_),node(X,Y)),t).
caseb(t,RID) :- holds(at(robot(RID),node(X,Y)),t), holds(at(shelf(SID),node(X,Y)),t), not holds(carries(RID,SID),t).
casec(t,RID) :- holds(at(robot(RID),node(X,Y)),t), holds(at(shelf(SID),node(X,Y)),t), holds(carries(RID,SID),t), not init(at(pickingstation(_), node(X,Y))).
cased(t,RID,RQID,Item,RQAMT) :- holds(carries(RID,SID),t), holds(at(robot(RID),node(X,Y)),t), holds(at(shelf(SID),node(X,Y)),t), init(at(pickingstation(ID),node(X, Y))), init(request(RQID, Item, ID, RQAMT)), init(on(Item, SID, Amt)).

% robot action decisions

{occurs(robot(RID),A,t):posscasea(RID,A,t)}1 :- casea(t-1,RID).
{occurs(robot(RID),A,t):posscaseb(RID,A,t)}1 :- caseb(t-1,RID).
{occurs(robot(RID),A,t):posscasec(RID,A,t)}1 :- casec(t-1,RID).
{occurs(robot(RID),A,t):posscased(RID,A,t)}1 :- cased(t-1,RID,_,_,_).

% heuristics for better performance

_heuristic(occurs(robot(RID),deliver(RQID, PRID, RQAMT),t), sign, 1) :- init(request(RQID, PRID, StationID, RQAMT)), cased(t-1,RID,_,_,_).

% action effects (borrowed from original and edited)

holds(at(robot(RID),node(X+DX,Y+DY)),t) :-  node(X+DX,Y+DY), holds(at(robot(RID),node(X,Y)),t-1), occurs(robot(RID),move(DX,DY),t).
holds(at(shelf(SID),node(X+DX,Y+DY)),t) :-  node(X+DX,Y+DY), holds(at(robot(RID),node(X,Y)),t-1), occurs(robot(RID),move(DX,DY),t), holds(carries(RID,SID),t-1).
holds(carries(RID,SID),t)           :-  holds(at(robot(RID),node(X,Y)),t), holds(at(shelf(SID),node(X,Y)),t), occurs(robot(RID),pickup,t).
moved(robot(RID),t)                 :-  occurs(robot(RID),move(_,_),t), robot(RID).
%holds(on(PRID,SID,AMT-RQAMT),t)     :-  AMT>RQAMT, holds(on(PRID,SID,AMT),t-1), holds(carries(RID,SID),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t).
holds(request(RQID,PRID,PSID,RQAMT-AMT),t):-    AMT<RQAMT, holds(request(RQID,PRID,PSID,RQAMT),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, AMT)), t).
holds(done(RQID,PRID,PSID,RQAMT),t) :-  holds(request(RQID,PRID,PSID,RQAMT),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t).

%%	INERTIA (borrowed from original)
holds(F,t) :- holds(F,t-1), not ab(F,t).

%%	INERTIA - ABNORMALS
%       Anything that changes is abnormal and won't be handled by inertia.
ab(at(robot(RID),node(X,Y)),t) 	:-	holds(at(robot(RID),node(X,Y)),t-1),
								moved(robot(RID),t).
ab(at(shelf(SID),node(X,Y)),t) 	:-	holds(at(shelf(SID),node(X,Y)),t-1),
								holds(carries(RID,SID),t-1),
								moved(robot(RID),t).
ab(carries(RID,SID),t)		:-	holds(carries(RID,SID),t-1),
								occurs(robot(RID),putdown,t).
ab(on(PRID,SID,AMT),t)		:-	holds(on(PRID,SID,AMT),t-1),
                               holds(carries(RID,SID),t-1),
                               occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t).
ab(request(RQID,PRID,PID,RQAMT),t) :-	holds(request(RQID,PRID,PID,RQAMT),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, AMT)), t).
ab(request(RQID,PRID,PID,RQAMT),t) :-	holds(done(RQID,PRID,PID,RQAMT),t).
ab(done(RQID,PRID,PID,RQAMT),t)    :-	holds(done(RQID,PRID,PID,RQAMT),t-1).


% relaxation 2 effects
holds(on(PRID,SID),t) :- holds(on(PRID,SID),t-1).
:- occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t), holds(carries(RID,SID),t), not init(on(PRID, SID)).

%%%%%%% constraints (partly borrowed from original) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%       loop prevention
:-  occurs(robot(RID),move(A,B),t), occurs(robot(RID),move(X,Y),t+1), occurs(robot(RID),move(-A,-B),t+2), occurs(robot(RID),move(-X,-Y),t+3).
:-  occurs(robot(RID),move(A,B),t), occurs(robot(RID),move(X,Y),t+1), occurs(robot(RID),move(-X,-Y),t+2), occurs(robot(RID),move(-A,-B),t+3).

%       there can only be one action per robot at a time
:-  occurs(robot(RID),A,t),occurs(robot(RID),B,t),A!=B.

%       no movement in circle
:-  occurs(robot(RID),move(A,B),t),occurs(robot(RID),move(C,D),t+1),occurs(robot(RID),move(E,F),t+2),occurs(robot(RID),move(G,H),t+3),holds(at(robot(RID),node(X,Y)),t-1),holds(at(robot(RID),node(X,Y)),t+3).

%       There can only be one robot and one shelf at a node at the same time.
:- holds(at(robot(I),node(X,Y)),T), holds(at(robot(J),node(X,Y)),T), I!=J.
:- holds(at(shelf(I),node(X,Y)),T), holds(at(shelf(J),node(X,Y)),T), I!=J.

%       A robot/shelf can only be at one node at a time.
:-  robot(RID), 2{holds(at(robot(RID),node(X,Y)),t) : node(X,Y)}.
:-  shelf(SID), 2{holds(at(shelf(SID),node(X,Y)),t) : node(X,Y)}.

%       Two robots can't swap places as they would crash.
:-  holds(at(robot(RID1),node(X1,Y1)),t), holds(at(robot(RID2),node(X2,Y2)),t),
    holds(at(robot(RID1),node(X2,Y2)),t-1), holds(at(robot(RID2),node(X1,Y1)),t-1),
    RID1<RID2, |X2-X1|+|Y2-Y1|==1.

%       On higway nodes shelfs can't be putdown and robots can't do nothing for more than one time step.
:-  highway(X,Y), not node(X,Y).
:-  highway(X,Y), holds(at(robot(RID),node(X,Y)),t), occurs(robot(RID),putdown,t).
:-  highway(X,Y), holds(at(robot(RID),node(X,Y)),t), not moved(robot(RID),t-1), not moved(robot(RID),t), not init(at(robot(RID),node(X,Y))).

%       Small scope loop preventions preventing opposite actions at adjacent time points.
:- occurs(robot(RID),move(DX,DY),t), occurs(robot(RID),move(-DX,-DY),t-1).
:- occurs(robot(RID),pickup,t), occurs(robot(RID),putdown,t-1).
:- occurs(robot(RID),putdown,t), occurs(robot(RID),pickup,t-1).

%%%%%%% iclingo %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#program check(t).
#external query(t).

%%%%%%% fulfillment of orders (borrowed from original) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

goal(t) :- not holds(request(_,_,_,_),t).
:- not goal(t), query(t).

%%%%%%% minimization %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%#minimize{RID,T : occurs(robot(RID),_,T)}.

%%%%%%% output %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

#program base.
#show occurs/3.
