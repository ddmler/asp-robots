% There can only be one robot at a node at the same time.
bug(1,RID1,A,T):-	node(X,Y), holds(at(robot(RID1),node(X,Y)),T), holds(at(robot(RID2),node(X,Y)),T), RID1<RID2, occurs(robot(RID1),A,T).

% a robot can not pick up a shelf, which was picked up before by another robot.
bug(2,RID1,T) :- not holds(carries(RID1,SID),T-1),holds(carries(RID1,SID),T),holds(carries(RID2,SID),T-1),holds(carries(RID2,SID),T),RID1<RID2.
bug(2,RID2,T) :- not holds(carries(RID2,SID),T-1),holds(carries(RID2,SID),T),holds(carries(RID1,SID),T-1),holds(carries(RID1,SID),T),RID2>RID1.

% Two robots can't swap places as they would crash.
bug(3,RID1,X1-X2,Y1-Y2,T):-	holds(at(robot(RID1),node(X1,Y1)),T), holds(at(robot(RID2),node(X2,Y2)),T),
	holds(at(robot(RID1),node(X2,Y2)),T-1), holds(at(robot(RID2),node(X1,Y1)),T-1),
	RID1<RID2, |X2-X1|+|Y2-Y1|==1.

% output
#show bug/3.
#show bug/4.
#show bug/5.
