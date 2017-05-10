

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
robot(Robot) :- self(Robot).
shelf(Shelf) :- init(at(shelf(Shelf), node(_,_))).












action(move(0,1)).
action(move(1,0)).
action(move(0,-1)).
action(move(-1,0)).
action(pickup).
action(putdown).
action(deliver(RQID, PRID, AMT)) :- init(request(RQID, PRID, StationID, RQAMT)), AMT = 1..RQAMT.

#program step(t).


%%cast internal plan format to output plan format (borrowed from original)
occurs(object(robot, RID), action(move, (DX,DY)), t) :- occurs(robot(RID),move(DX,DY),t).
occurs(object(robot, RID), action(pickup, ()), t) :- occurs(robot(RID),pickup,t).
occurs(object(robot, RID), action(putdown, ()), t) :- occurs(robot(RID),putdown,t).
occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t) :- occurs(robot(RID),deliver(RQID, PRID, RQAMT),t).



{ occurs(robot(RID),A,t):action(A) } 1 :- self(RID).




% action effects (borrowed from original and edited)

holds(at(robot(RID),node(X+DX,Y+DY)),t) :-  node(X+DX,Y+DY), holds(at(robot(RID),node(X,Y)),t-1), occurs(robot(RID),move(DX,DY),t).
holds(at(shelf(SID),node(X+DX,Y+DY)),t) :-  node(X+DX,Y+DY), holds(at(robot(RID),node(X,Y)),t-1), occurs(robot(RID),move(DX,DY),t), holds(carries(RID,SID),t-1).
holds(carries(RID,SID),t)           :-  holds(at(robot(RID),node(X,Y)),t), holds(at(shelf(SID),node(X,Y)),t), occurs(robot(RID),pickup,t).
moved(robot(RID),t)                 :-  occurs(robot(RID),move(_,_),t), robot(RID).
holds(on(PRID,SID,AMT-RQAMT),t)     :-  AMT>RQAMT, holds(on(PRID,SID,AMT),t-1), holds(carries(RID,SID),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t).
holds(request(RQID,PRID,PSID,RQAMT-AMT),t):-    AMT<RQAMT, holds(request(RQID,PRID,PSID,RQAMT),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, AMT)), t).
holds(done(RQID,PRID,PSID,RQAMT),t) :-  holds(request(RQID,PRID,PSID,RQAMT),t-1), occurs(object(robot, RID), action(deliver, (RQID, PRID, RQAMT)), t).

%%  INERTIA (borrowed from original)
holds(F,t) :- holds(F,t-1), not ab(F,t).

%%  INERTIA - ABNORMALS
%       Anything that changes is abnormal and won't be handled by inertia.
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



#program check(t).
#external query(t).
%%%%%%% fulfillment of orders (borrowed from original) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

orderdelivered(t) :- not holds(request(_,_,_,_),t).

%orderdelivered(t+1) :- orderdelivered(t).

putback(t) :- orderdelivered(t-1), holds(at(shelf(SID),node(X,Y)),t), init(object(shelf,SID),value(at,(X,Y))), not holds(carries(_,SID),t).

goal(t) :- putback(t).
:- not goal(t), query(t).


#show occurs/3.
