%       loop prevention
bug(1,RID,T):-  occurs(robot(RID),move(A,B),T), occurs(robot(RID),move(X,Y),T+1), occurs(robot(RID),move(-A,-B),T+2), occurs(robot(RID),move(-X,-Y),T+3).
bug(1,RID,T):-  occurs(robot(RID),move(A,B),T), occurs(robot(RID),move(X,Y),T+1), occurs(robot(RID),move(-X,-Y),T+2), occurs(robot(RID),move(-A,-B),T+3).

%       no movement in circle
bug(2,RID,T):-  occurs(robot(RID),move(A,B),T),occurs(robot(RID),move(C,D),T+1),occurs(robot(RID),move(E,F),T+2),occurs(robot(RID),move(G,H),T+3),holds(at(robot(RID),node(X,Y)),T-1),holds(at(robot(RID),node(X,Y)),T+3).

%       There can only be one robot and one shelf at a node at the same time.
bug(3,RID1,T):-	node(X,Y), holds(at(robot(RID1),node(X,Y)),T), holds(at(robot(RID2),node(X,Y)),T), RID1<RID2.
bug(4,SID1,T):-	node(X,Y), holds(at(shelf(SID1),node(X,Y)),T), holds(at(shelf(SID2),node(X,Y)),T), SID1<SID2.


%       Two robots can'T swap places as they would crash.
bug(5,RID1,T):-	holds(at(robot(RID1),node(X1,Y1)),T), holds(at(robot(RID2),node(X2,Y2)),T),
	holds(at(robot(RID1),node(X2,Y2)),T-1), holds(at(robot(RID2),node(X1,Y1)),T-1),
	RID1<RID2, |X2-X1|+|Y2-Y1|==1.

%       On higway nodes shelfs can'T be putdown and robots can'T do nothing for more than one time step.
bug(6,1,0):-  highway(X,Y), not node(X,Y).
bug(7,RID,T):-  highway(X,Y), holds(at(robot(RID),node(X,Y)),T), occurs(robot(RID),putdown,T).
bug(8,RID,T):-  highway(X,Y), holds(at(robot(RID),node(X,Y)),T), not moved(robot(RID),T-1), not moved(robot(RID),T), not init(at(robot(RID),node(X,Y))).

%       Small scope loop preventions preventing opposite actions at adjacent time points.
bug(9,RID,T):- occurs(robot(RID),move(DX,DY),T), occurs(robot(RID),move(-DX,-DY),T-1).
bug(10,RID,T):- occurs(robot(RID),pickup,T), occurs(robot(RID),putdown,T-1).
bug(11,RID,T):- occurs(robot(RID),putdown,T), occurs(robot(RID),pickup,T-1).

#show bug/1.
