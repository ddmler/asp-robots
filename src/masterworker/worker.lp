% INITIAL STATE
holds(F,0) :- init(F).

% cast external instance format to internal instance format
node(X,Y) :- init(object(node, ID), value(at, (X, Y))).
highway(X,Y) :- init(object(highway, ID), value(at, (X, Y))).
init(at(robot(ID), node(X, Y))) :- init(object(robot, ID), value(at, (X, Y))), self(ID).
init(at(shelf(ID), node(X, Y))) :- init(object(shelf, ID), value(at, (X, Y))).
init(at(pickingstation(ID), node(X, Y))) :- init(object(pickingStation, ID), value(at, (X, Y))).
init(on(ID, ShelfID, Amount)) :- init(object(product, ID), value(on, (ShelfID, Amount))).
init(request(ID, Item, StationID, Amount)) :- init(object(order, ID), value(line, (Item, Amount))), init(object(order,ID),value(pickingStation, StationID)).
init(carries(RID,SID)) :- init(object(robot, RID), value(carries, SID)).
robot(Robot) :- self(Robot).
shelf(Shelf) :- init(at(shelf(Shelf), node(_,_))).

% ACTIONS
action(move(0,1)).
action(move(1,0)).
action(move(0,-1)).
action(move(-1,0)).
action(pickup).
action(putdown).
action(deliver(RQID, PRID, AMT)) :- init(request(RQID, PRID, StationID, RQAMT)), AMT = 1..RQAMT.

#program step(t).


% cast internal plan format to output plan format (borrowed from original)
occurs(object(robot, RID), action(move, (DX,DY)), t) :- occurs(robot(RID),move(DX,DY),t).
occurs(object(robot, RID), action(pickup, ()), t) :- occurs(robot(RID),pickup,t).
occurs(object(robot, RID), action(putdown, ()), t) :- occurs(robot(RID),putdown,t).
occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t) :- occurs(robot(RID),deliver(RQID, PRID, RQAMT),t).

% define all possible acions for a robot at step t for each case a to d

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
posscased(RID,deliver(RQID, PRID, AMT),t) :- robot(RID), init(request(RQID, PRID, StationID, RQAMT)), AMT = 1..RQAMT.

% defining cases for different actions
% case a: If there is no shelf at the robot's position (x,y), the robot can only move and wait
casea(t,RID) :- holds(at(robot(RID),node(X,Y)),t), not holds(at(shelf(_),node(X,Y)),t).

% case b: If there is a shelf at the robot's position (x,y) and the robot does not carry this shelf, the robot can move and pickup and wait
caseb(t,RID) :- holds(at(robot(RID),node(X,Y)),t), holds(at(shelf(SID),node(X,Y)),t), not holds(carries(RID,SID),t).

% case c: If there is a shelf at the robot's position (x,y) and the robot carries this shelf and there is no picking station at this position, the robot can move and putdown and wait
casec(t,RID) :- holds(at(robot(RID),node(X,Y)),t), holds(at(shelf(SID),node(X,Y)),t), holds(carries(RID,SID),t), not init(at(pickingstation(_), node(X,Y))).

% case d: If there is a shelf at the robot's position (x,y) and the robot carries this shelf and there is a picking station at this position, the robot can move and deliver and wait
cased(t,RID,RQID,Item,Amt) :- holds(carries(RID,SID),t), holds(at(robot(RID),node(X,Y)),t), holds(at(shelf(SID),node(X,Y)),t), init(at(pickingstation(ID),node(X, Y))), init(request(RQID, Item, ID, Amount)), init(on(Item, SID, Amt)).

% robot action decisions are selected through the appropriate case
{occurs(robot(RID),A,t):posscasea(RID,A,t)}1 :- casea(t-1,RID).
{occurs(robot(RID),A,t):posscaseb(RID,A,t)}1 :- caseb(t-1,RID).
{occurs(robot(RID),A,t):posscasec(RID,A,t)}1 :- casec(t-1,RID).
{occurs(robot(RID),A,t):posscased(RID,A,t)}1 :- cased(t-1,RID,_,_,_).

% handle bugs from checker
% If bug 1 occurs, the robot is not allowed to do the action A, which causes the bug
:- bug(1, RID, A, t), occurs(robot(RID), A, t), self(RID).

% If bug 2 occurs, the robot must not pickup
:- bug(2,RID,t),occurs(robot(RID), pickup, t),self(RID).

% If bug 3 occurs, the robot must not move in direction (X,Y)
:- bug(3,RID,X,Y,t), occurs(robot(RID),move(X,Y),t), self(RID).

% action effects (borrowed from original and edited)
holds(at(robot(RID),node(X+DX,Y+DY)),t) :-  node(X+DX,Y+DY), holds(at(robot(RID),node(X,Y)),t-1), occurs(robot(RID),move(DX,DY),t).
holds(at(shelf(SID),node(X+DX,Y+DY)),t) :-  node(X+DX,Y+DY), holds(at(robot(RID),node(X,Y)),t-1), occurs(robot(RID),move(DX,DY),t), holds(carries(RID,SID),t-1).
holds(carries(RID,SID),t)           :-  holds(at(robot(RID),node(X,Y)),t), holds(at(shelf(SID),node(X,Y)),t), occurs(robot(RID),pickup,t).
moved(robot(RID),t)                 :-  occurs(robot(RID),move(_,_),t), robot(RID).
holds(on(PRID,SID,AMT-RQAMT),t)     :-  AMT>RQAMT, holds(on(PRID,SID,AMT),t-1), holds(carries(RID,SID),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t).
holds(request(RQID,PRID,PSID,RQAMT-AMT),t):-    AMT<RQAMT, holds(request(RQID,PRID,PSID,RQAMT),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, AMT)), t).
holds(done(RQID,PRID,PSID,RQAMT),t) :-  holds(request(RQID,PRID,PSID,RQAMT),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t).

% INERTIA (borrowed from original)
holds(F,t) :- holds(F,t-1), not ab(F,t).

% INERTIA - ABNORMALS
% Anything that changes is abnormal and won't be handled by inertia.
ab(at(robot(RID),node(X,Y)),t)  :-  holds(at(robot(RID),node(X,Y)),t-1),
                                moved(robot(RID),t).
ab(at(shelf(SID),node(X,Y)),t)  :-  holds(at(shelf(SID),node(X,Y)),t-1),
                                holds(carries(RID,SID),t-1),
                                moved(robot(RID),t).
ab(carries(RID,SID),t)      :-  holds(carries(RID,SID),t-1),
                                occurs(robot(RID),putdown,t).
ab(on(PRID,SID,AMT),t)      :-  holds(on(PRID,SID,AMT),t-1),
                                holds(carries(RID,SID),t-1),
                                occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t).
ab(request(RQID,PRID,PID,RQAMT),t) :-   holds(request(RQID,PRID,PID,RQAMT),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, AMT)), t).
ab(request(RQID,PRID,PID,RQAMT),t) :-   holds(done(RQID,PRID,PID,RQAMT),t).
ab(done(RQID,PRID,PID,RQAMT),t) :- holds(done(RQID,PRID,PID,RQAMT),t-1).

% Constraints (borrowed and edited from original)
:-  highway(X,Y), holds(at(robot(RID),node(X,Y)),t), occurs(robot(RID),putdown,t), self(RID).
:-  highway(X,Y), holds(at(robot(RID),node(X,Y)),t), not moved(robot(RID),t-1), not moved(robot(RID),t), not init(at(robot(RID),node(X,Y))), self(RID).
:- occurs(robot(RID),move(DX,DY),t), occurs(robot(RID),move(-DX,-DY),t-1), self(RID).
:- occurs(robot(RID),pickup,t), occurs(robot(RID),putdown,t-1), self(RID).
:- occurs(robot(RID),putdown,t), occurs(robot(RID),pickup,t-1), self(RID).
:-  occurs(robot(RID),move(A,B),t), occurs(robot(RID),move(X,Y),t+1), occurs(robot(RID),move(-A,-B),t+2), occurs(robot(RID),move(-X,-Y),t+3).
:-  occurs(robot(RID),move(A,B),t), occurs(robot(RID),move(X,Y),t+1), occurs(robot(RID),move(-X,-Y),t+2), occurs(robot(RID),move(-A,-B),t+3).
:- holds(at(shelf(SID1),node(X,Y)),t), holds(at(shelf(SID2),node(X,Y)),t), SID1!=SID2.

#program check(t).
#external query(t).
%%%%%%% fulfillment of orders %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% check, if robot delivered all his orders
orderdelivered(RQID,t) :- not holds(request(RQID,_,_,_),t), roborder(RID,RQID), self(RID).

% check, if robot has returned his shelf to its original position
putback(RQID, t) :- orderdelivered(RQID, t-1), holds(at(shelf(SID),node(X,Y)),t), init(object(shelf,SID),value(at,(X,Y))), occurs(object(robot, _), action(putdown, ()), t).

% check, if there are requests left
requestleft(t) :- holds(request(RQID, _, _, _),t), roborder(RID, RQID), self(RID).
allorderdone(t) :- not requestleft(t).
allputback(t) :- putback(_, t), allorderdone(t-1).

allorderdone(t) :- allorderdone(t-1).
allputback(t) :- allputback(t-1).
orderdelivered(RQID, t) :- orderdelivered(RQID, t-1).

% there can not be a plan without fulfillment of all orders and putback
:- roborder(RID, RQID), self(RID), not orderdelivered(RQID, t), query(t).

% if a robot has no orders assigned, it can relax
goal(1) :- not roborder(RID, _), self(RID).

% the robot has to return all shelves
goal(t) :- allputback(t).
:- not goal(t), query(t).

% output
#show occurs/3.
#show holds/2.
#show node/2.
#show orderdelivered/2  .
#show highway/2.
#show moved/2.
#show init/1.
#show putback/2.
